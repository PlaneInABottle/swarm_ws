import rclpy
import traceback
import time
import math
from enum import Enum, auto
from threading import Lock
from typing import Dict, Optional, Type, Any, cast # Added cast for type checking

# Import specific rclpy types for hints
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.action.server import ActionServer, ServerGoalHandle # Import ServerGoalHandle for type hinting
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

import lifecycle_msgs.srv
import lifecycle_msgs.msg

# Import PX4 messages
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleRatesSetpoint # Example if needed for other handlers
)

# Import Action types from interfaces package
# Replace 'swarm_interfaces' with your actual interface package name
from swarm_interfaces.action import Takeoff

from drone_swarm_control.utils.types import InternalState
# Import Handlers (assuming they are in ../handlers relative to this file)
# Use try-except for imports if handlers are optional or might not exist initially
try:
    from ..handlers.core_action_handler_base import ActionHandlerBase
    from ..handlers.takeoff_action_handler import TakeoffActionHandler
    # Import other handlers similarly...
    # from ..handlers.land_action_handler import LandActionHandler
    # from ..handlers.navigate_to_point_action_handler import NavigateToPointActionHandler
    # from ..handlers.execute_dynamic_motion_handler import ExecuteDynamicMotionHandler
except ImportError as e:
    # Handle the case where handlers might not be found if structure changes
    # This allows the node to potentially still load, but handlers won't work.
    print(f"Error importing handlers: {e}. Ensure handlers exist in the correct directory.")
    # Define dummy classes if needed to prevent NameErrors later, or raise error
    class ActionHandlerBase: pass # Dummy base
    class TakeoffActionHandler(ActionHandlerBase): pass # Dummy handler


# --- Constants ---
DEFAULT_HEARTBEAT_PERIOD = 0.1 # seconds (10 Hz) for OffboardControlMode
SETPOINT_PUBLISH_PERIOD = 0.05 # seconds (20 Hz) for TrajectorySetpoint - MUST BE > 2Hz
CONNECTION_TIMEOUT_SEC = 5.0  # Timeout for waiting for PX4 connection (status)



class CoreDroneController(LifecycleNode):
    """
    Core Drone Controller ROS 2 Lifecycle Node.
    Manages direct PX4 communication (Offboard Mode, Setpoints, Commands)
    and hosts Action Servers via dedicated Action Handler classes.
    Uses an internal state machine and continuous setpoint publishing for robustness.
    """
    def __init__(self, **kwargs: Any):
        """ Initializes the CoreDroneController node. """
        super().__init__('core_drone_controller', **kwargs)
        self.get_logger().info("Creating Core Drone Controller...")

        # --- Parameters (Declare defaults, get in on_configure) ---
        self.declare_parameter('target_system_id', 2)
        self.declare_parameter('px4_ns_prefix', 'px4_1') # Example, should match drone
        self.declare_parameter('offboard_heartbeat_period', DEFAULT_HEARTBEAT_PERIOD)
        self.declare_parameter('setpoint_publish_period', SETPOINT_PUBLISH_PERIOD)
        self.declare_parameter('enable_debug_logging', False)

        # --- Internal State ---
        self._target_system_id: int = 2
        self._px4_ns_prefix: str = 'px4_1'
        self._heartbeat_period: float = DEFAULT_HEARTBEAT_PERIOD
        self._setpoint_period: float = SETPOINT_PUBLISH_PERIOD
        self._debug_logging: bool = False

        self._vehicle_status: Optional[VehicleStatus] = None
        self._vehicle_local_position: Optional[VehicleLocalPosition] = None
        self._is_armed: bool = False # Derived from status
        self._is_offboard: bool = False # Derived from status
        self._is_on_ground_approx: bool = True # Estimated from status/heuristics

        # Internal State Machine
        self._internal_state: InternalState = InternalState.UNKNOWN
        self._state_lock = Lock() # Lock for thread-safe state access/modification
        self._active_goal_handle: Optional[ServerGoalHandle] = None # Store the currently active goal

        # Desired Setpoint State (updated by handlers/hold_position)
        self._desired_setpoint: Optional[TrajectorySetpoint] = None

        # --- Publishers, Subscribers, Timers (Initialized to None) ---
        self._offboard_control_mode_pub: Optional[Publisher] = None
        self._trajectory_setpoint_pub: Optional[Publisher] = None
        self._vehicle_command_pub: Optional[Publisher] = None
        self._vehicle_status_sub: Optional[Subscription] = None
        self._vehicle_local_position_sub: Optional[Subscription] = None
        self._offboard_heartbeat_timer: Optional[Timer] = None
        self._setpoint_publishing_timer: Optional[Timer] = None

        # --- Action Handlers ---
        self._handlers: dict[str, ActionHandlerBase] = {}

        # Instantiate handlers (They require self and logger)
        # The logger is available *after* super().__init__
        node_logger = self.get_logger()
        # Use try-except during instantiation if a handler class might be missing
        try:
            self._handlers['takeoff'] = TakeoffActionHandler(self, node_logger)
            # self._handlers['land'] = LandActionHandler(self, node_logger)
            # self._handlers['navigate_to_point'] = NavigateToPointActionHandler(self, node_logger)
            # self._handlers['execute_dynamic_motion'] = ExecuteDynamicMotionHandler(self, node_logger)
            # Add other handlers...
        except NameError as e:
             self.get_logger().error(f"Failed to instantiate handlers, class likely missing: {e}")
             # Node will continue but actions won't work

        self.get_logger().info("Core Drone Controller node created.")

    # === State Management ===
    def set_internal_state(self, new_state: InternalState, goal_handle: Optional[ServerGoalHandle] = None) -> None:
        """
        Thread-safe method to change the internal operational state.
        Automatically commands hold when transitioning to idle states.
        """
        with self._state_lock:
            if self._internal_state != new_state:
                old_state_name = self._internal_state.name
                self._internal_state = new_state
                self.get_logger().info(f"Internal state changing: {old_state_name} -> {self._internal_state.name}")

                # If transitioning TO an idle/holding state, ensure hold is commanded
                if new_state in [InternalState.IDLE_HOVERING, InternalState.IDLE_ON_GROUND]:
                     self.get_logger().debug("Transitioning to idle/hold state, setting desired setpoint to HOLD.")
                     self._active_goal_handle = None # Clear active goal when idle/holding
                     # Update desired setpoint immediately for the publisher timer
                     self._update_desired_setpoint_to_hold() # Ignore return value here, timer will handle errors

                elif new_state == InternalState.EXECUTING_ACTION:
                     if goal_handle is None:
                          self.get_logger().error("Cannot transition to EXECUTING_ACTION without a goal handle! Reverting state.")
                          # Revert state to prevent inconsistent state
                          self._internal_state = InternalState.IDLE_HOVERING # Fallback to hover
                          self._active_goal_handle = None
                     else:
                          self._active_goal_handle = goal_handle
                          self.get_logger().info(f"Now executing goal: {goal_handle.goal_id.uuid.hex}")
                elif new_state == InternalState.ARMING:
                     self._active_goal_handle = goal_handle # Store goal handle associated with arming if needed
                else:
                     # Handle other state transitions if needed
                     self._active_goal_handle = None # Clear goal handle for other states unless needed

    def get_internal_state(self) -> InternalState:
         """Thread-safe method to get the internal state."""
         with self._state_lock:
             return self._internal_state

    def get_active_goal_handle(self) -> Optional[ServerGoalHandle]:
         """Thread-safe method to get the active goal handle."""
         with self._state_lock:
             return self._active_goal_handle

    # === Lifecycle Implementation ===

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Configures resources: parameters, pubs/subs, handlers, timers. """
        self.get_logger().info("Configuring Core Drone Controller...")
        # Reset state variables that might persist from previous runs
        self._vehicle_status = None
        self._vehicle_local_position = None
        self._is_armed = False
        self._is_offboard = False
        self._is_on_ground_approx = True
        self._desired_setpoint = None
        self._active_goal_handle = None

        try:
            # Get parameters
            self._target_system_id = int(self.get_parameter('target_system_id').value)
            self._px4_ns_prefix = str(self.get_parameter('px4_ns_prefix').value)
            self._heartbeat_period = float(self.get_parameter('offboard_heartbeat_period').value)
            self._setpoint_period = float(self.get_parameter('setpoint_publish_period').value)
            self._debug_logging = bool(self.get_parameter('enable_debug_logging').value)
            self.get_logger().info(f"Params: SystemID={self._target_system_id}, Prefix='{self._px4_ns_prefix}', "
                                   f"Heartbeat={self._heartbeat_period:.3f}s, Setpoint={self._setpoint_period:.3f}s")

            if self._setpoint_period > 0.45: # Check if setpoint rate is > ~2.2Hz
                 self.get_logger().warn("Setpoint publish period is > 0.45s. PX4 Offboard mode requires > 2Hz!")
                 # Allow configuration but warn heavily. Could return FAILURE here.

            # Define QoS Profiles
            qos_pub_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
            qos_sub_status = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
            qos_sub_pose = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=5)

            # Create Publishers
            fmu_in_base = f"/{self._px4_ns_prefix}/fmu/in"
            self._offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f"{fmu_in_base}/offboard_control_mode", qos_pub_reliable)
            self._trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f"{fmu_in_base}/trajectory_setpoint", qos_pub_reliable)
            self._vehicle_command_pub = self.create_publisher(VehicleCommand, f"{fmu_in_base}/vehicle_command", qos_pub_reliable)
            self.get_logger().info("Publishers created.")

            # Create Subscribers
            fmu_out_base = f"/{self._px4_ns_prefix}/fmu/out"
            self._vehicle_status_sub = self.create_subscription(VehicleStatus, f"{fmu_out_base}/vehicle_status_v1", self._vehicle_status_callback, qos_sub_status)
            self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, f"{fmu_out_base}/vehicle_local_position", self._vehicle_local_position_callback, qos_sub_pose)
            self.get_logger().info("Subscribers created.")

            # Wait briefly for initial status to ensure connection
            wait_start = time.monotonic()
            while self._vehicle_status is None and (time.monotonic() - wait_start) < CONNECTION_TIMEOUT_SEC:
                 self.get_logger().info("Waiting for initial VehicleStatus message...", throttle_duration_sec=1.0)
                 rclpy.spin_once(self, timeout_sec=0.1) # Okay in configure/activate
            if self._vehicle_status is None:
                 self.get_logger().error(f"Timeout waiting for VehicleStatus from {fmu_out_base}/vehicle_status. Check PX4 connection & namespace.")
                 self._cleanup_resources()
                 return TransitionCallbackReturn.FAILURE # Use FAILURE for configuration issues

            # Start Action Handlers
            if not self._handlers:
                 self.get_logger().warn("No action handlers were instantiated.")
            for name, handler in self._handlers.items():
                # Check if handler is a dummy due to import error
                if isinstance(handler, ActionHandlerBase) and handler.__class__ != ActionHandlerBase:
                    self.get_logger().info(f"Starting handler: {name}")
                    if not handler.start(): # Call the handler's start() method
                        self.get_logger().error(f"Failed to start handler: {name}")
                        self._cleanup_resources() # Attempt cleanup before failing configure
                        return TransitionCallbackReturn.FAILURE # Indicate configuration failed
                else:
                     self.get_logger().warn(f"Skipping start for handler '{name}', possibly due to import error or base class instance.")


            # Create Heartbeat Timer (paused)
            self._offboard_heartbeat_timer = self.create_timer(self._heartbeat_period, self._publish_offboard_control_mode_heartbeat)
            self._offboard_heartbeat_timer.cancel()

            # Create Setpoint Publishing Timer (paused)
            self._setpoint_publishing_timer = self.create_timer(self._setpoint_period, self._publish_current_setpoint)
            self._setpoint_publishing_timer.cancel()
            self.get_logger().info("Timers created.")

            # Set initial internal state
            self.set_internal_state(InternalState.IDLE_ON_GROUND)

            self.get_logger().info("Core Drone Controller configured successfully.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Exception during configuration: {e}")
            self.get_logger().error(traceback.format_exc())
            self._cleanup_resources() # Attempt cleanup
            return TransitionCallbackReturn.ERROR # Use ERROR for unexpected exceptions

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Activates the node: starts timers and sets initial state. """
        self.get_logger().info("Activating Core Drone Controller...")
        try:
            # Sanity check: Ensure needed components exist from configure phase
            if None in [self._offboard_heartbeat_timer, self._setpoint_publishing_timer,
                        self._vehicle_status_sub, self._vehicle_local_position_sub,
                        self._offboard_control_mode_pub, self._trajectory_setpoint_pub,
                        self._vehicle_command_pub]:
                 self.get_logger().error("Cannot activate: Not all components were created during configure.")
                 return TransitionCallbackReturn.FAILURE

            # Start the heartbeat timer
            self.get_logger().info("Starting offboard heartbeat timer.")
            # Use cast to satisfy type checker, we checked for None above
            cast(Timer, self._offboard_heartbeat_timer).reset()


            # Start the setpoint publishing timer
            self.get_logger().info(f"Starting setpoint publishing timer ({1.0/self._setpoint_period:.1f} Hz).")
            # Initialize desired setpoint to current pos before starting timer
            if not self._update_desired_setpoint_to_hold():
                 self.get_logger().warn("Could not get initial position for hold on activation. Timer will attempt to recover.")
            # Use cast to satisfy type checker
            cast(Timer, self._setpoint_publishing_timer).reset()


            # Send initial offboard mode command (PX4 needs this stream before accepting mode change)
            # The heartbeat timer handles the continuous stream.
            # Consider sending VEHICLE_CMD_DO_SET_MODE here or let first action handle it.
            # self.engage_offboard_mode() # Optional: Send immediately on activation

            # Update internal state based on actual drone status
            if self.is_armed():
                 self.set_internal_state(InternalState.IDLE_HOVERING)
            else:
                 self.set_internal_state(InternalState.IDLE_ON_GROUND)

            self.get_logger().info("Core Drone Controller activated.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during activation: {e}")
            self.get_logger().error(traceback.format_exc())
            # Attempt to stop timers if they were started
            if self._offboard_heartbeat_timer: self._offboard_heartbeat_timer.cancel()
            if self._setpoint_publishing_timer: self._setpoint_publishing_timer.cancel()
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Deactivates the node: stops timers, commands hold. """
        self.get_logger().info("Deactivating Core Drone Controller...")
        try:
            # Stop timers first
            if self._offboard_heartbeat_timer:
                 self.get_logger().info("Stopping offboard heartbeat timer.")
                 self._offboard_heartbeat_timer.cancel()
            if self._setpoint_publishing_timer:
                 self.get_logger().info("Stopping setpoint publishing timer.")
                 self._setpoint_publishing_timer.cancel()

            # Set desired state to HOLD before publishing final setpoint
            self.get_logger().info("Setting desired setpoint to HOLD on deactivation.")
            hold_set = self._update_desired_setpoint_to_hold()

            # Publish the hold setpoint once immediately for responsiveness
            if hold_set and self._desired_setpoint is not None:
                 self._desired_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                 if self._trajectory_setpoint_pub:
                     self._trajectory_setpoint_pub.publish(self._desired_setpoint)
                     self.get_logger().info("Published final hold setpoint.")
                 else:
                     self.get_logger().warn("Could not publish final hold setpoint: publisher missing.")
            elif not hold_set:
                 self.get_logger().warn("Could not determine hold position on deactivation.")

            # Set internal state appropriately
            if self.is_armed():
                 self.set_internal_state(InternalState.IDLE_HOVERING) # Still armed but inactive
            else:
                 self.set_internal_state(InternalState.IDLE_ON_GROUND)

            # Consider canceling active goals?
            active_goal = self.get_active_goal_handle()
            if active_goal and active_goal.is_active:
                 self.get_logger().warn(f"Node deactivated with active goal {active_goal.goal_id.uuid.hex}. Goal not automatically cancelled by node.")
                 # Decide if node should force cancel goals on deactivate.
                 # active_goal.abort() # Or active_goal.cancel_goal() -> requires client interaction

            self.get_logger().info("Core Drone Controller deactivated.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during deactivation: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Cleans up resources: stops timers, sends final hold, cleans ROS entities. """
        self.get_logger().info("Cleaning up Core Drone Controller...")
        # Ensure timers are stopped (should be done by deactivate or error handling)
        if self._offboard_heartbeat_timer and not self._offboard_heartbeat_timer.is_canceled():
            self._offboard_heartbeat_timer.cancel()
        if self._setpoint_publishing_timer and not self._setpoint_publishing_timer.is_canceled():
            self._setpoint_publishing_timer.cancel()

        # Send a final safe command if possible before cleanup
        if self._trajectory_setpoint_pub is not None:
            # Try to update to hold, then publish once
            if self._update_desired_setpoint_to_hold() and self._desired_setpoint:
                 self._desired_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                 try:
                      self._trajectory_setpoint_pub.publish(self._desired_setpoint)
                      self.get_logger().info("Published final hold setpoint during cleanup.")
                 except Exception:
                      self.get_logger().warn("Failed to publish final hold setpoint during cleanup.")
        # Consider disarm if appropriate for cleanup
        # if self._vehicle_command_pub and self.is_armed():
        #      self.disarm()

        # Cleanup ROS resources
        cleanup_success = self._cleanup_resources()

        # Reset internal state
        self.set_internal_state(InternalState.UNKNOWN)

        self.get_logger().info("Core Drone Controller cleanup finished.")
        return TransitionCallbackReturn.SUCCESS if cleanup_success else TransitionCallbackReturn.WARNING

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Handles node shutdown, performing cleanup. """
        self.get_logger().info("Shutting down Core Drone Controller...")
        # Perform cleanup, similar to on_cleanup
        return self.on_cleanup(state)

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Handles errors during lifecycle transitions. """
        self.get_logger().error(f"Lifecycle Error Transition triggered from state {state.label}!")
        # Perform cleanup on error
        self._cleanup_resources()
        self.set_internal_state(InternalState.UNKNOWN) # Reset state
        return TransitionCallbackReturn.SUCCESS # Allow transition to finalize/unconfigured

    def _cleanup_resources(self) -> bool:
        """Helper method to destroy ROS resources and stop handlers."""
        success = True
        self.get_logger().info("Executing resource cleanup...")
        try:
            # Stop Action Handlers first (might need access to pubs/subs)
            for name, handler in self._handlers.items():
                 # Check if handler is a real handler instance before stopping
                 if isinstance(handler, ActionHandlerBase) and handler.__class__ != ActionHandlerBase:
                     self.get_logger().info(f"Stopping handler: {name}")
                     handler.stop() # Call the handler's stop method

            # Destroy timers
            if self._offboard_heartbeat_timer:
                self.get_logger().info("Destroying heartbeat timer.")
                self.destroy_timer(self._offboard_heartbeat_timer)
            if self._setpoint_publishing_timer:
                self.get_logger().info("Destroying setpoint timer.")
                self.destroy_timer(self._setpoint_publishing_timer)
            self._offboard_heartbeat_timer = None
            self._setpoint_publishing_timer = None

            # Destroy publishers
            pubs = {'ocm': self._offboard_control_mode_pub, 'traj': self._trajectory_setpoint_pub, 'cmd': self._vehicle_command_pub}
            for name, pub in pubs.items():
                if pub:
                    self.get_logger().debug(f"Destroying publisher {name}")
                    self.destroy_publisher(pub)
            self._offboard_control_mode_pub = self._trajectory_setpoint_pub = self._vehicle_command_pub = None

             # Destroy subscribers
            subs = {'status': self._vehicle_status_sub, 'lpos': self._vehicle_local_position_sub}
            for name, sub in subs.items():
                 if sub:
                    self.get_logger().debug(f"Destroying subscriber {name}")
                    self.destroy_subscription(sub)
            self._vehicle_status_sub = self._vehicle_local_position_sub = None

            self.get_logger().info("Resource cleanup helper finished.")

        except Exception as e:
            self.get_logger().error(f"Exception during _cleanup_resources: {e}")
            self.get_logger().error(traceback.format_exc())
            success = False
        finally:
             # Reset state variables (already done in cleanup caller)
             self._vehicle_status = None
             self._vehicle_local_position = None
             self._is_armed = False
             self._is_offboard = False
             self._is_on_ground_approx = True
             self._desired_setpoint = None
             self._active_goal_handle = None
        return success

    # === PX4 Communication Callbacks ===

    def _vehicle_status_callback(self, msg: VehicleStatus):
        """Stores status and updates internal flags and state machine."""
        if self._debug_logging:
            # Log relevant fields based on the provided definition
            self.get_logger().debug(f"Status: NAV={msg.nav_state}, ARM={msg.arming_state}, HIL={msg.hil_state}, Failsafe={msg.failsafe}", throttle_duration_sec=5)
        self._vehicle_status = msg
        new_armed_state = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        new_offboard_state = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        # --- On-Ground Estimation Logic (Updated) ---
        # Heuristic based ONLY on arming_state since landed_state is missing
        new_on_ground_state = not new_armed_state # Assume ON_GROUND if DISARMED, IN_AIR if ARMED
        if not hasattr(self, '_warned_missing_landed_state'): # Log warning only once
             self.get_logger().warn("VehicleStatus message missing 'landed_state'. Estimating on_ground based SOLELY on arming state (DISARMED=GROUND, ARMED=AIR). This is less reliable.")
             self._warned_missing_landed_state = True # Prevent repeated warnings
        # --- End of Updated On-Ground Logic ---


        # Update internal state machine based on changes (use lock for safety)
        with self._state_lock:
            current_internal_state = self._internal_state # Get state inside lock

            # Update primary flags
            if new_armed_state != self._is_armed:
                self.get_logger().info(f"Arming state changed: {self._is_armed} -> {new_armed_state}")
                self._is_armed = new_armed_state

            if new_offboard_state != self._is_offboard:
                 self.get_logger().info(f"Offboard mode changed: {self._is_offboard} -> {new_offboard_state}")
                 self._is_offboard = new_offboard_state
                 # If we exited offboard unexpectedly while trying to execute/hover
                 if not new_offboard_state and current_internal_state in [InternalState.IDLE_HOVERING, InternalState.EXECUTING_ACTION, InternalState.ARMING]:
                      self.get_logger().error("Exited OFFBOARD mode unexpectedly! Transitioning to IDLE_HOVERING (will attempt hold).")
                      # Force state change, hold_position will be commanded by timer default
                      self._internal_state = InternalState.IDLE_HOVERING
                      self._active_goal_handle = None # Clear active goal

            if new_on_ground_state != self._is_on_ground_approx:
                 self.get_logger().info(f"On-ground state changed (estimated): {self._is_on_ground_approx} -> {new_on_ground_state}")
                 self._is_on_ground_approx = new_on_ground_state


            # Update state machine based on combined flags (only if IDLE)
            if current_internal_state in [InternalState.IDLE_ON_GROUND, InternalState.IDLE_HOVERING]:
                if self._is_armed and not self._is_on_ground_approx and current_internal_state == InternalState.IDLE_ON_GROUND:
                    self.get_logger().info("State change detected: Armed and In-Air -> IDLE_HOVERING")
                    self._internal_state = InternalState.IDLE_HOVERING
                    self._update_desired_setpoint_to_hold()
                elif not self._is_armed and self._is_on_ground_approx and current_internal_state == InternalState.IDLE_HOVERING:
                    self.get_logger().info("State change detected: Disarmed and On-Ground -> IDLE_ON_GROUND")
                    self._internal_state = InternalState.IDLE_ON_GROUND
                    self._update_desired_setpoint_to_hold()
                elif not self._is_armed and current_internal_state == InternalState.IDLE_HOVERING:
                     # Disarmed while hovering? Transition to on_ground state
                     self.get_logger().warn("Disarmed while IDLE_HOVERING. Transitioning to IDLE_ON_GROUND.")
                     self._internal_state = InternalState.IDLE_ON_GROUND
                     self._update_desired_setpoint_to_hold() # Command hold at last known pos

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Stores the latest vehicle local position."""
        if self._debug_logging:
             self.get_logger().debug(f"Local Pos: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}, VZ={msg.vz:.2f}, H={msg.heading:.2f}", throttle_duration_sec=5)
        self._vehicle_local_position = msg

    def _publish_offboard_control_mode_heartbeat(self):
        """Publishes the OffboardControlMode message periodically."""
        if self._offboard_control_mode_pub is None: return
        msg = OffboardControlMode()
        # Set flags based on what handlers might command. Default to position.
        msg.position = True
        msg.velocity = False # Set true if handlers use velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        try:
            self._offboard_control_mode_pub.publish(msg)
        except Exception as e:
             self.get_logger().error(f"Failed to publish offboard control mode: {e}", throttle_duration_sec=5)

    def _publish_current_setpoint(self):
        """
        Periodically publishes the `self._desired_setpoint`.
        Defaults to holding current position if no desired setpoint is set.
        This runs continuously when the node is ACTIVE.
        """
        if self._trajectory_setpoint_pub is None: return

        publish_setpoint: Optional[TrajectorySetpoint] = None
        # Access shared state within lock for consistency
        with self._state_lock:
             publish_setpoint = self._desired_setpoint

        # If no desired setpoint, default to HOLD
        if publish_setpoint is None:
             if self._update_desired_setpoint_to_hold(): # Tries to update self._desired_setpoint
                  with self._state_lock: # Re-acquire lock to read potentially updated value
                       publish_setpoint = self._desired_setpoint
             else:
                  self.get_logger().warn("No desired setpoint and cannot get current position. Publishing NaN setpoint.", throttle_duration_sec=5)
                  # Publish NaNs as a fallback if position is unknown
                  nan_setpoint = TrajectorySetpoint()
                  nan_setpoint.position = [float('nan')] * 3
                  nan_setpoint.yaw = float('nan')
                  nan_setpoint.velocity = [float('nan')] * 3
                  nan_setpoint.acceleration = [float('nan')] * 3
                  nan_setpoint.jerk = [float('nan')] * 3
                  nan_setpoint.yawspeed = float('nan')
                  publish_setpoint = nan_setpoint

        # Publish the determined setpoint
        if publish_setpoint:
            publish_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            try:
                self._trajectory_setpoint_pub.publish(publish_setpoint)
                if self._debug_logging:
                     pos = publish_setpoint.position
                     yaw = publish_setpoint.yaw
                     self.get_logger().debug(f"Pub Setpoint: N={pos[0]:.2f}, E={pos[1]:.2f}, D={pos[2]:.2f}, Yaw={yaw:.2f}", throttle_duration_sec=1)
            except Exception as e:
                 self.get_logger().error(f"Failed to publish trajectory setpoint: {e}", throttle_duration_sec=5)

    # === Public Methods for Action Handlers ===

    def is_armed(self) -> bool:
        """Returns the last known armed state."""
        # Ensure status is recent? Add timestamp check if needed.
        return self._is_armed

    def is_on_ground(self) -> bool:
        """Returns the estimated on-ground state."""
        return self._is_on_ground_approx

    def get_current_local_position(self) -> Optional[VehicleLocalPosition]:
        """Returns the last known VehicleLocalPosition message."""
        # Ensure position is recent? Add timestamp check if needed.
        # e.g., if self._vehicle_local_position and (self.get_clock().now().nanoseconds - self._vehicle_local_position.timestamp_sample * 1000) < POSITION_TIMEOUT_NS:
        # return self._vehicle_local_position else return None
        return self._vehicle_local_position

    def get_vehicle_status(self) -> Optional[VehicleStatus]:
         """Returns the last known VehicleStatus message."""
         return self._vehicle_status

    def set_target_setpoint(self, x: float = float('nan'), y: float = float('nan'), z: float = float('nan'), yaw: float = float('nan')):
        """
        Called by handlers to update the desired setpoint state.
        The setpoint timer will periodically publish this. Thread-safe.
        """
        with self._state_lock: # Protect access to shared _desired_setpoint
            # Create or update the TrajectorySetpoint message safely
            if self._desired_setpoint is None:
                self._desired_setpoint = TrajectorySetpoint(
                    # Initialize all fields to NaN by default
                    position = [float('nan')] * 3,
                    velocity = [float('nan')] * 3,
                    acceleration = [float('nan')] * 3,
                    jerk = [float('nan')] * 3,
                    yaw = float('nan'),
                    yawspeed = float('nan')
                )

            # Update only the relevant fields for position control
            self._desired_setpoint.position = [float(x), float(y), float(z)]
            self._desired_setpoint.yaw = float(yaw)
            # Ensure velocity/accel/rates are NaN if using position control
            self._desired_setpoint.velocity = [float('nan')] * 3
            self._desired_setpoint.acceleration = [float('nan')] * 3
            self._desired_setpoint.jerk = [float('nan')] * 3
            self._desired_setpoint.yawspeed = float('nan')
            if self._debug_logging:
                self.get_logger().debug(f"Desired setpoint updated: N={x:.2f}, E={y:.2f}, D={z:.2f}, Yaw={yaw:.2f}")

    def hold_position(self) -> bool:
        """
        Updates the desired setpoint state to hold the current position.
        Returns True if successful in setting the state, False otherwise. Thread-safe.
        """
        self.get_logger().debug("Setting desired setpoint to HOLD current position.")
        # Lock is acquired inside the helper
        return self._update_desired_setpoint_to_hold()

    def _update_desired_setpoint_to_hold(self) -> bool:
         """Internal helper to update self._desired_setpoint to current pos. Thread-safe."""
         # This method might be called from different threads (state machine, timer)
         # It needs to access _vehicle_local_position (updated by sub callback)
         # and update _desired_setpoint (read by timer callback)
         # -> Use lock
         current_pos = self._vehicle_local_position # Read LPOS within lock
         if current_pos is not None:
            # Update the TrajectorySetpoint message for holding
            if self._desired_setpoint is None:
                self._desired_setpoint = TrajectorySetpoint() # Create if doesn't exist

            self._desired_setpoint.position = [current_pos.x, current_pos.y, current_pos.z]
            # Set velocities/accel/rates to NaN to ensure position control
            self._desired_setpoint.velocity = [float('nan')] * 3
            self._desired_setpoint.acceleration = [float('nan')] * 3
            self._desired_setpoint.jerk = [float('nan')] * 3
            # Hold current yaw if available, otherwise let PX4 control it
            self._desired_setpoint.yaw = current_pos.heading if current_pos.heading_good_for_control else float('nan')
            self._desired_setpoint.yawspeed = float('nan')
            if self._debug_logging:
                self.get_logger().debug(f"Desired setpoint updated to HOLD: N={current_pos.x:.2f}, E={current_pos.y:.2f}, D={current_pos.z:.2f}, Yaw={self._desired_setpoint.yaw:.2f}")
            return True
         else:
            self.get_logger().warning("Cannot update desired setpoint to Hold: Current local position unknown.", throttle_duration_sec=2.0)
            # Clear desired setpoint? Or leave previous? Clearing will trigger NaN publish.
            # self._desired_setpoint = None # Setting to None triggers NaN publish in timer
            return False

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0, param4: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0) -> bool:
        """Helper to publish VehicleCommand. Assumes publisher exists."""
        if self._vehicle_command_pub is None:
            self.get_logger().error(f"Cannot publish VehicleCommand {command}: Publisher not available.")
            return False

        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = self._target_system_id
        msg.target_component = 1 # Autopilot
        msg.source_system = 255 # Companion computer ID
        msg.source_component = 190 # MAV_COMP_ID_ONBOARD_COMPUTER or similar
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        try:
            self._vehicle_command_pub.publish(msg)
            self.get_logger().info(f"Published VehicleCommand: {command} ({param1=:.1f}, {param2=:.1f}...)")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish vehicle command {command}: {e}", throttle_duration_sec=5)
            return False

    def arm(self) -> bool:
        """Sends the ARM command."""
        self.get_logger().info("Sending ARM command...")
        # Optionally check state before sending?
        # if self.get_internal_state() != InternalState.IDLE_ON_GROUND:
        #     self.get_logger().warn("Arm command sent while not in IDLE_ON_GROUND state.")
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self) -> bool:
        """Sends the DISARM command."""
        self.get_logger().info("Sending DISARM command...")
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def land(self) -> bool:
        """Sends the LAND command (sets mode to LAND)."""
        self.get_logger().info("Sending LAND command (changing mode)...")
        # Optionally update internal state here? Or let status callback handle it?
        # self.set_internal_state(InternalState.LANDING_AUTO)
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def engage_offboard_mode(self) -> bool:
        """Sends command to switch to Offboard flight mode."""
        # PX4 requires continuous setpoints BEFORE this command is accepted.
        # Ensure setpoint timer is running and sending valid data.
        self.get_logger().info("Sending SET_MODE=OFFBOARD command...")
        # Mode 6 = Offboard
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)


# === Main Execution Logic ===
def main(args: Optional[list[str]] = None):
     """ Main function to run the node standalone. """
     print('Starting Core Drone Controller Node...')
     rclpy.init(args=args)
     node: Optional[CoreDroneController] = None
     executor: Optional[MultiThreadedExecutor] = None
     try:
         node = CoreDroneController()
         executor = MultiThreadedExecutor()
         executor.add_node(node)

         # Minimal manual transition for testing:
         node.get_logger().info("Attempting to configure node...")
         configure_result = node.trigger_configure() # Check return value
         if configure_result == TransitionCallbackReturn.SUCCESS:
              # Spin briefly to allow configuration callbacks and status checks
              executor.spin_once(timeout_sec=1.0) # Spin AFTER triggering
              # Now check the state using the service call method (more robust)
              # This requires the node's services to be available
              cli = node.create_client(lifecycle_msgs.srv.GetState, f'{node.get_name()}/get_state')
              if not cli.wait_for_service(timeout_sec=2.0):
                   node.get_logger().error("GetState service not available after configure.")
                   raise RuntimeError("Could not verify node state.")
              req = lifecycle_msgs.srv.GetState.Request()
              future = cli.call_async(req)
              rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
              if not future.done() or future.result() is None:
                   node.get_logger().error("Failed to get node state after configure.")
                   raise RuntimeError("Could not verify node state.")

              current_state_id = future.result().current_state.id
              if current_state_id == lifecycle_msgs.msg.State.PRIMARY_STATE_INACTIVE:
                   node.get_logger().info("Configuration successful (INACTIVE state confirmed), activating...")
                   activate_result = node.trigger_activate()
                   if activate_result == TransitionCallbackReturn.SUCCESS:
                        executor.spin_once(timeout_sec=1.0) # Allow activate callbacks
                        # Optionally verify active state here too using service call
                   else:
                        node.get_logger().error(f"Activation trigger failed! Result: {activate_result}")
                        raise RuntimeError("Activation trigger failed during startup.")
              else:
                   # Configuration reported success via return code but state is not INACTIVE
                   node.get_logger().error(f"Configuration trigger returned SUCCESS but node state is {current_state_id}, not INACTIVE.")
                   raise RuntimeError("Inconsistent state after configuration.")
              node.destroy_client(cli) # Clean up client
         else:
              node.get_logger().error(f"Configuration trigger failed! Result: {configure_result}. Shutting down.")
              raise RuntimeError("Configuration trigger failed during startup.")

         node.get_logger().info('Core Drone Controller spinning...')
         executor.spin()

     except KeyboardInterrupt:
         if node: node.get_logger().info('Keyboard interrupt, shutting down...')
         else: print('Keyboard interrupt, shutting down...')
     except Exception as e:
          if node: node.get_logger().error(f"Exception in main spin: {e}")
          else: print(f"Exception before node init: {e}")
          traceback.print_exc()
     finally:
         # Orderly shutdown
         if node and executor and rclpy.ok():
             node.get_logger().info('Initiating node shutdown sequence...')
             # Simplified shutdown: just trigger transitions downwards
             # Checking state during shutdown after errors can be unreliable
             try:
                 node.get_logger().info('Attempting Deactivate...')
                 node.trigger_deactivate()
                 executor.spin_once(timeout_sec=1.0)
             except Exception as e:
                 node.get_logger().error(f"Error during deactivate trigger: {e}")
             try:
                 node.get_logger().info('Attempting Cleanup...')
                 node.trigger_cleanup()
                 executor.spin_once(timeout_sec=1.0)
             except Exception as e:
                 node.get_logger().error(f"Error during cleanup trigger: {e}")
             try:
                 node.get_logger().info('Attempting Shutdown...')
                 node.trigger_shutdown()
                 executor.spin_once(timeout_sec=1.0)
             except Exception as e:
                 node.get_logger().error(f"Error during shutdown trigger: {e}")


             node.get_logger().info('Shutting down executor...')
             executor.shutdown()

         if rclpy.ok():
             print('Shutting down RCLPY...')
             rclpy.shutdown()
         print('Shutdown complete.')


if __name__ == '__main__':
     main()
