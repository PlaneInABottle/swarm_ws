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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

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
        self.declare_parameter('enable_debug_logging', True) # Default to True for easier debugging setup

        # --- Internal State ---
        self._target_system_id: int = 2
        self._px4_ns_prefix: str = 'px4_1'
        self._heartbeat_period: float = DEFAULT_HEARTBEAT_PERIOD
        self._setpoint_period: float = SETPOINT_PUBLISH_PERIOD
        self._debug_logging: bool = False # Disabled to reduce CPU usage

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
            # DEBUG: Log handler instantiation attempt
            if self._debug_logging: node_logger.debug("Attempting to instantiate TakeoffActionHandler...")
            self._handlers['takeoff'] = TakeoffActionHandler(self, node_logger)
            if self._debug_logging: node_logger.debug("TakeoffActionHandler instantiated.")
            # self._handlers['land'] = LandActionHandler(self, node_logger)
            # self._handlers['navigate_to_point'] = NavigateToPointActionHandler(self, node_logger)
            # self._handlers['execute_dynamic_motion'] = ExecuteDynamicMotionHandler(self, node_logger)
            # Add other handlers...
        except NameError as e:
             node_logger.error(f"Failed to instantiate handlers, class likely missing: {e}")
             # Node will continue but actions won't work

        self.get_logger().info("Core Drone Controller node created.")
        if self._debug_logging: self.get_logger().debug("Node __init__ complete.")

    # === State Management ===
    def set_internal_state(self, new_state: InternalState, goal_handle: Optional[ServerGoalHandle] = None) -> None:
        """
        Thread-safe method to change the internal operational state.
        Automatically commands hold when transitioning to idle states.
        """
        with self._state_lock:
            old_state = self._internal_state
            if old_state != new_state:
                old_state_name = old_state.name
                self._internal_state = new_state
                goal_id_str = f" (Goal: {goal_handle.goal_id})" if goal_handle else ""
                self.get_logger().info(f"Internal state changing: {old_state_name} -> {new_state.name}{goal_id_str}")
                if self._debug_logging: self.get_logger().debug(f"set_internal_state: {old_state_name} -> {new_state.name}{goal_id_str}")

                # If transitioning TO an idle/holding state, ensure hold is commanded
                if new_state in [InternalState.IDLE_HOVERING, InternalState.IDLE_ON_GROUND]:
                    if self._debug_logging: self.get_logger().debug(f"Transitioning to {new_state.name}, ensuring HOLD setpoint. Clearing active goal.")
                    self._active_goal_handle = None # Clear active goal when idle/holding
                    # Update desired setpoint immediately for the publisher timer
                    hold_success = self._update_desired_setpoint_to_hold() # Ignore return value here, timer will handle errors
                    if self._debug_logging: self.get_logger().debug(f"Hold setpoint update attempt result: {hold_success}")

                elif new_state == InternalState.EXECUTING_ACTION:
                    if goal_handle is None:
                        self.get_logger().error("Cannot transition to EXECUTING_ACTION without a goal handle! Reverting state.")
                        # Revert state to prevent inconsistent state
                        self._internal_state = InternalState.IDLE_HOVERING # Fallback to hover
                        self._active_goal_handle = None
                        if self._debug_logging: self.get_logger().debug("Error: No goal handle for EXECUTING_ACTION. Reverted to IDLE_HOVERING.")
                    else:
                        self._active_goal_handle = goal_handle
                        if self._debug_logging: self.get_logger().debug(f"Now executing goal: {goal_handle.goal_id}")
                elif new_state == InternalState.ARMING:
                    self._active_goal_handle = goal_handle # Store goal handle associated with arming if needed
                    if self._debug_logging: self.get_logger().debug(f"Entered ARMING state. Goal: {goal_id_str}")
                else:
                    # Handle other state transitions if needed
                    if self._debug_logging: self.get_logger().debug(f"Entered {new_state.name} state. Clearing active goal.")
                    self._active_goal_handle = None # Clear goal handle for other states unless needed
            # else: # No state change
            #     if self._debug_logging: self.get_logger().debug(f"set_internal_state called with same state: {new_state.name}", throttle_duration_sec=10.0)


    def get_internal_state(self) -> InternalState:
         """Thread-safe method to get the internal state."""
         with self._state_lock:
             # Optional: Add debug log if needed, but can be noisy
             # if self._debug_logging: self.get_logger().debug(f"get_internal_state returning: {self._internal_state.name}", throttle_duration_sec=5.0)
             return self._internal_state

    def get_active_goal_handle(self) -> Optional[ServerGoalHandle]:
         """Thread-safe method to get the active goal handle."""
         with self._state_lock:
             goal_id_str = f"{self._active_goal_handle.goal_id}" if self._active_goal_handle else "None"
             if self._debug_logging: self.get_logger().debug(f"get_active_goal_handle returning: {goal_id_str}", throttle_duration_sec=5.0)
             return self._active_goal_handle

    # === Lifecycle Implementation ===

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Configures resources: parameters, pubs/subs, handlers, timers. """
        self.get_logger().info("Configuring Core Drone Controller...")
        if self._debug_logging: self.get_logger().debug("on_configure: Entering configuration...")

        # Reset state variables that might persist from previous runs
        self._vehicle_status = None
        self._vehicle_local_position = None
        self._is_armed = False
        self._is_offboard = False
        self._is_on_ground_approx = True
        self._desired_setpoint = None
        self._active_goal_handle = None
        if self._debug_logging: self.get_logger().debug("on_configure: Internal state variables reset.")

        try:
            # Get parameters
            self._target_system_id = int(self.get_parameter('target_system_id').value)
            self._px4_ns_prefix = str(self.get_parameter('px4_ns_prefix').value)
            self._heartbeat_period = float(self.get_parameter('offboard_heartbeat_period').value)
            self._setpoint_period = float(self.get_parameter('setpoint_publish_period').value)
            # Debug logging disabled to reduce CPU usage
            self._debug_logging = False
            self.get_logger().info("Debug logging disabled to reduce CPU usage")
            self.get_logger().info(f"Params: SystemID={self._target_system_id}, Prefix='{self._px4_ns_prefix}', "
                                   f"Heartbeat={self._heartbeat_period:.3f}s, Setpoint={self._setpoint_period:.3f}s")
            if self._debug_logging: self.get_logger().debug("on_configure: Parameters retrieved.")

            if self._setpoint_period > 0.45: # Check if setpoint rate is > ~2.2Hz
                 self.get_logger().warn("Setpoint publish period is > 0.45s (Rate < ~2.2Hz). PX4 Offboard mode requires > 2Hz!")
                 if self._debug_logging: self.get_logger().debug("on_configure: Setpoint rate warning check passed/logged.")

            # Define QoS Profiles
            qos_pub_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
            qos_sub_status = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
            qos_sub_pose = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=5)
            if self._debug_logging: self.get_logger().debug(f"on_configure: QoS Profiles defined: PubReliable={qos_pub_reliable}, SubStatus={qos_sub_status}, SubPose={qos_sub_pose}")

            # Create Publishers
            fmu_in_base = f"/{self._px4_ns_prefix}/fmu/in"
            self._offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f"{fmu_in_base}/offboard_control_mode", qos_pub_reliable)
            self._trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f"{fmu_in_base}/trajectory_setpoint", qos_pub_reliable)
            self._vehicle_command_pub = self.create_publisher(VehicleCommand, f"{fmu_in_base}/vehicle_command", qos_pub_reliable)
            self.get_logger().info("Publishers created.")
            if self._debug_logging: self.get_logger().debug(f"on_configure: Publishers created for topics under {fmu_in_base}/")

            # Create Subscribers
            fmu_out_base = f"/{self._px4_ns_prefix}/fmu/out"
            self._vehicle_status_sub = self.create_subscription(VehicleStatus, f"{fmu_out_base}/vehicle_status_v1", self._vehicle_status_callback, qos_sub_status)             
            self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, f"{fmu_out_base}/vehicle_local_position", self._vehicle_local_position_callback, qos_sub_pose)
            self.get_logger().info("Subscribers created.")
            if self._debug_logging: self.get_logger().debug(f"on_configure: Subscribers created for topics under {fmu_out_base}/")


            # Start Action Handlers
            if not self._handlers:
                 self.get_logger().warn("No action handlers were instantiated.")
                 if self._debug_logging: self.get_logger().debug("on_configure: No action handlers found to start.")
            for name, handler in self._handlers.items():
                 # Check if handler is a dummy due to import error
                if isinstance(handler, ActionHandlerBase) and handler.__class__!= ActionHandlerBase:
                     self.get_logger().info(f"Starting handler: {name}")
                     if self._debug_logging: self.get_logger().debug(f"on_configure: Attempting to start handler '{name}'...")
                     if not handler.start(): # Pass group here
                         self.get_logger().error(f"Failed to start handler: {name}")
                         self._cleanup_resources() # Attempt cleanup before failing configure
                         if self._debug_logging: self.get_logger().debug(f"on_configure: FAILURE - Handler '{name}' failed to start.")
                         return TransitionCallbackReturn.FAILURE # Indicate configuration failed
                     else:
                         if self._debug_logging: self.get_logger().debug(f"on_configure: Handler '{name}' started successfully.")
                else:
                      self.get_logger().warn(f"Skipping start for handler '{name}', possibly due to import error or base class instance.")
                      if self._debug_logging: self.get_logger().debug(f"on_configure: Skipped starting handler '{name}'.")


            # Create Heartbeat Timer (paused)
            self._offboard_heartbeat_timer = self.create_timer(self._heartbeat_period, self._publish_offboard_control_mode_heartbeat)
            self._offboard_heartbeat_timer.cancel()
            if self._debug_logging: self.get_logger().debug(f"on_configure: Offboard heartbeat timer created (period: {self._heartbeat_period:.3f}s, initially cancelled).")


            # Create Setpoint Publishing Timer (paused)
            self._setpoint_publishing_timer = self.create_timer(self._setpoint_period, self._publish_current_setpoint)
            self._setpoint_publishing_timer.cancel()
            self.get_logger().info("Timers created.")
            if self._debug_logging: self.get_logger().debug(f"on_configure: Setpoint publishing timer created (period: {self._setpoint_period:.3f}s, initially cancelled).")


            # Set initial internal state based on potentially received status
            initial_state = InternalState.IDLE_ON_GROUND if self._is_on_ground_approx else InternalState.IDLE_HOVERING
            self.set_internal_state(initial_state)
            if self._debug_logging: self.get_logger().debug(f"on_configure: Initial internal state set to {initial_state.name} based on ground status ({self._is_on_ground_approx}).")


            self.get_logger().info("Core Drone Controller configured successfully.")
            if self._debug_logging: self.get_logger().debug("on_configure: SUCCESS")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Exception during configuration: {e}")
            self.get_logger().error(traceback.format_exc())
            if self._debug_logging: self.get_logger().debug(f"on_configure: ERROR - Exception occurred: {e}")
            self._cleanup_resources() # Attempt cleanup
            return TransitionCallbackReturn.ERROR # Use ERROR for unexpected exceptions

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Activates the node: starts timers and sets initial state. """
        self.get_logger().info("Activating Core Drone Controller...")
        if self._debug_logging: self.get_logger().debug("on_activate: Entering activation...")
        try:
            # Sanity check: Ensure needed components exist from configure phase
            if self._debug_logging: self.get_logger().debug("on_activate: Checking for required components...")
            if None in [self._offboard_heartbeat_timer, self._setpoint_publishing_timer,
                        self._vehicle_status_sub, self._vehicle_local_position_sub,
                        self._offboard_control_mode_pub, self._trajectory_setpoint_pub,
                        self._vehicle_command_pub]:
                 self.get_logger().error("Cannot activate: Not all components were created during configure.")
                 if self._debug_logging: self.get_logger().debug("on_activate: FAILURE - Missing required components.")
                 return TransitionCallbackReturn.FAILURE
            if self._debug_logging: self.get_logger().debug("on_activate: Component check passed.")

            # Start the heartbeat timer
            self.get_logger().info("Starting offboard heartbeat timer.")
            # Use cast to satisfy type checker, we checked for None above
            cast(Timer, self._offboard_heartbeat_timer).reset()
            if self._debug_logging: self.get_logger().debug("on_activate: Offboard heartbeat timer reset (started).")


            # Start the setpoint publishing timer
            self.get_logger().info(f"Starting setpoint publishing timer ({1.0/self._setpoint_period:.1f} Hz).")
            # Initialize desired setpoint to current pos before starting timer
            if self._debug_logging: self.get_logger().debug("on_activate: Attempting to set initial HOLD setpoint...")
            hold_success = self._update_desired_setpoint_to_hold()
            if not hold_success:
                 self.get_logger().warn("Could not get initial position for hold on activation. Timer will attempt to recover.")
            if self._debug_logging: self.get_logger().debug(f"on_activate: Initial HOLD setpoint update result: {hold_success}")
            # Use cast to satisfy type checker
            cast(Timer, self._setpoint_publishing_timer).reset()
            if self._debug_logging: self.get_logger().debug("on_activate: Setpoint publishing timer reset (started).")


            # Send initial offboard mode command (PX4 needs this stream before accepting mode change)
            # The heartbeat timer handles the continuous stream.
            # Consider sending VEHICLE_CMD_DO_SET_MODE here or let first action handle it.
            # self.engage_offboard_mode() # Optional: Send immediately on activation
            if self._debug_logging: self.get_logger().debug("on_activate: Note: Offboard heartbeat started, but engage_offboard_mode() not called automatically on activation.")

            # Update internal state based on actual drone status (might have changed since configure)
            if self._debug_logging: self.get_logger().debug("on_activate: Updating internal state based on current arm/ground status...")
            if self.is_armed() and not self.is_on_ground():
                 self.set_internal_state(InternalState.IDLE_HOVERING)
                 if self._debug_logging: self.get_logger().debug("on_activate: Set state to IDLE_HOVERING (Armed & In-Air).")
            else:
                 self.set_internal_state(InternalState.IDLE_ON_GROUND)
                 if self._debug_logging: self.get_logger().debug("on_activate: Set state to IDLE_ON_GROUND (Disarmed or On-Ground).")

            self.get_logger().info("Core Drone Controller activated.")
            if self._debug_logging: self.get_logger().debug("on_activate: SUCCESS")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during activation: {e}")
            self.get_logger().error(traceback.format_exc())
            if self._debug_logging: self.get_logger().debug(f"on_activate: ERROR - Exception occurred: {e}")
            # Attempt to stop timers if they were started
            if self._offboard_heartbeat_timer: self._offboard_heartbeat_timer.cancel()
            if self._setpoint_publishing_timer: self._setpoint_publishing_timer.cancel()
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Deactivates the node: stops timers, commands hold. """
        self.get_logger().info("Deactivating Core Drone Controller...")
        if self._debug_logging: self.get_logger().debug("on_deactivate: Entering deactivation...")
        try:
            # Stop timers first
            if self._offboard_heartbeat_timer:
                 self.get_logger().info("Stopping offboard heartbeat timer.")
                 self._offboard_heartbeat_timer.cancel()
                 if self._debug_logging: self.get_logger().debug("on_deactivate: Offboard heartbeat timer cancelled.")
            if self._setpoint_publishing_timer:
                 self.get_logger().info("Stopping setpoint publishing timer.")
                 self._setpoint_publishing_timer.cancel()
                 if self._debug_logging: self.get_logger().debug("on_deactivate: Setpoint publishing timer cancelled.")

            # Set desired state to HOLD before publishing final setpoint
            self.get_logger().info("Setting desired setpoint to HOLD on deactivation.")
            if self._debug_logging: self.get_logger().debug("on_deactivate: Attempting to set HOLD setpoint...")
            hold_set = self._update_desired_setpoint_to_hold()
            if self._debug_logging: self.get_logger().debug(f"on_deactivate: HOLD setpoint update result: {hold_set}")

            # Publish the hold setpoint once immediately for responsiveness
            if hold_set and self._desired_setpoint is not None:
                 self._desired_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                 if self._trajectory_setpoint_pub:
                     self._trajectory_setpoint_pub.publish(self._desired_setpoint)
                     self.get_logger().info("Published final hold setpoint.")
                     if self._debug_logging: self.get_logger().debug("on_deactivate: Final HOLD setpoint published.")
                 else:
                     self.get_logger().warn("Could not publish final hold setpoint: publisher missing.")
                     if self._debug_logging: self.get_logger().debug("on_deactivate: Could not publish final HOLD: publisher missing.")
            elif not hold_set:
                 self.get_logger().warn("Could not determine hold position on deactivation.")
                 if self._debug_logging: self.get_logger().debug("on_deactivate: Could not publish final HOLD: failed to determine position.")

            # Set internal state appropriately (reflecting timers stopped but drone state unchanged)
            if self._debug_logging: self.get_logger().debug("on_deactivate: Setting internal state based on current arm/ground status...")
            if self.is_armed() and not self.is_on_ground():
                 self.set_internal_state(InternalState.IDLE_HOVERING) # Still armed but inactive node
                 if self._debug_logging: self.get_logger().debug("on_deactivate: Set internal state to IDLE_HOVERING.")
            else:
                 self.set_internal_state(InternalState.IDLE_ON_GROUND) # Disarmed or on ground
                 if self._debug_logging: self.get_logger().debug("on_deactivate: Set internal state to IDLE_ON_GROUND.")

            # Consider canceling active goals?
            active_goal = self.get_active_goal_handle()
            if active_goal and active_goal.is_active:
                 self.get_logger().warn(f"Node deactivated with active goal {active_goal.goal_id}. Goal not automatically cancelled by node.")
                 if self._debug_logging: self.get_logger().debug(f"on_deactivate: Warning - Active goal {active_goal.goal_id} remains.")
                 # Decide if node should force cancel goals on deactivate.
                 # active_goal.abort() # Or active_goal.cancel_goal() -> requires client interaction

            self.get_logger().info("Core Drone Controller deactivated.")
            if self._debug_logging: self.get_logger().debug("on_deactivate: SUCCESS")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during deactivation: {e}")
            self.get_logger().error(traceback.format_exc())
            if self._debug_logging: self.get_logger().debug(f"on_deactivate: ERROR - Exception occurred: {e}")
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Cleans up resources: stops timers, sends final hold, cleans ROS entities. """
        self.get_logger().info("Cleaning up Core Drone Controller...")
        if self._debug_logging: self.get_logger().debug("on_cleanup: Entering cleanup...")
        # Ensure timers are stopped (should be done by deactivate or error handling)
        if self._offboard_heartbeat_timer and not self._offboard_heartbeat_timer.is_canceled():
            if self._debug_logging: self.get_logger().debug("on_cleanup: Heartbeat timer was not cancelled, cancelling now.")
            self._offboard_heartbeat_timer.cancel()
        if self._setpoint_publishing_timer and not self._setpoint_publishing_timer.is_canceled():
            if self._debug_logging: self.get_logger().debug("on_cleanup: Setpoint timer was not cancelled, cancelling now.")
            self._setpoint_publishing_timer.cancel()

        # Send a final safe command if possible before cleanup (Best effort)
        if self._trajectory_setpoint_pub is not None:
            if self._debug_logging: self.get_logger().debug("on_cleanup: Attempting to publish final HOLD setpoint...")
            # Try to update to hold, then publish once
            hold_set = self._update_desired_setpoint_to_hold()
            if self._debug_logging: self.get_logger().debug(f"on_cleanup: HOLD setpoint update result: {hold_set}")
            if hold_set and self._desired_setpoint:
                 self._desired_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                 try:
                      self._trajectory_setpoint_pub.publish(self._desired_setpoint)
                      self.get_logger().info("Published final hold setpoint during cleanup.")
                      if self._debug_logging: self.get_logger().debug("on_cleanup: Final HOLD setpoint published.")
                 except Exception as pub_e:
                      self.get_logger().warn(f"Failed to publish final hold setpoint during cleanup: {pub_e}")
                      if self._debug_logging: self.get_logger().debug(f"on_cleanup: Exception publishing final HOLD: {pub_e}")
            else:
                if self._debug_logging: self.get_logger().debug("on_cleanup: Did not publish final HOLD (could not determine pos or publisher missing).")
        # Consider disarm if appropriate for cleanup
        # if self._vehicle_command_pub and self.is_armed():
        #     if self._debug_logging: self.get_logger().debug("on_cleanup: Attempting DISARM.")
        #     self.disarm()

        # Cleanup ROS resources
        if self._debug_logging: self.get_logger().debug("on_cleanup: Calling _cleanup_resources...")
        cleanup_success = self._cleanup_resources()
        if self._debug_logging: self.get_logger().debug(f"on_cleanup: _cleanup_resources returned: {cleanup_success}")


        # Reset internal state
        self.set_internal_state(InternalState.UNKNOWN)
        if self._debug_logging: self.get_logger().debug("on_cleanup: Internal state set to UNKNOWN.")


        self.get_logger().info("Core Drone Controller cleanup finished.")
        if self._debug_logging: self.get_logger().debug(f"on_cleanup: {'SUCCESS' if cleanup_success else 'WARNING'}")
        return TransitionCallbackReturn.SUCCESS if cleanup_success else TransitionCallbackReturn.WARNING

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Handles node shutdown, performing cleanup. """
        self.get_logger().info("Shutting down Core Drone Controller...")
        if self._debug_logging: self.get_logger().debug(f"on_shutdown: Entering shutdown from state {state.label}...")
        # Perform cleanup, similar to on_cleanup
        result = self.on_cleanup(state)
        if self._debug_logging: self.get_logger().debug(f"on_shutdown: Cleanup result: {result}. Shutting down.")
        return result

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Handles errors during lifecycle transitions. """
        self.get_logger().error(f"Lifecycle Error Transition triggered from state {state.label}!")
        if self._debug_logging: self.get_logger().debug(f"on_error: Entering error handler from state {state.label}...")
        # Perform cleanup on error
        cleanup_success = self._cleanup_resources()
        if self._debug_logging: self.get_logger().debug(f"on_error: Cleanup result: {cleanup_success}.")
        self.set_internal_state(InternalState.UNKNOWN) # Reset state
        if self._debug_logging: self.get_logger().debug("on_error: Internal state set to UNKNOWN. Returning SUCCESS to allow finalization.")
        return TransitionCallbackReturn.SUCCESS # Allow transition to finalize/unconfigured

    def _cleanup_resources(self) -> bool:
        """Helper method to destroy ROS resources and stop handlers."""
        success = True
        self.get_logger().info("Executing resource cleanup...")
        if self._debug_logging: self.get_logger().debug("_cleanup_resources: Starting...")
        try:
            # Stop Action Handlers first (might need access to pubs/subs)
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Stopping action handlers...")
            for name, handler in self._handlers.items():
                 # Check if handler is a real handler instance before stopping
                 if isinstance(handler, ActionHandlerBase) and handler.__class__ != ActionHandlerBase:
                     self.get_logger().info(f"Stopping handler: {name}")
                     if self._debug_logging: self.get_logger().debug(f"_cleanup_resources: Stopping handler '{name}'...")
                     handler.stop() # Call the handler's stop method
                     if self._debug_logging: self.get_logger().debug(f"_cleanup_resources: Handler '{name}' stopped.")
                 else:
                     if self._debug_logging: self.get_logger().debug(f"_cleanup_resources: Skipping stop for handler '{name}'.")


            # Destroy timers
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Destroying timers...")
            if self._offboard_heartbeat_timer:
                 if self._debug_logging: self.get_logger().debug("_cleanup_resources: Destroying heartbeat timer.")
                 self.destroy_timer(self._offboard_heartbeat_timer)
            if self._setpoint_publishing_timer:
                 if self._debug_logging: self.get_logger().debug("_cleanup_resources: Destroying setpoint timer.")
                 self.destroy_timer(self._setpoint_publishing_timer)
            self._offboard_heartbeat_timer = None
            self._setpoint_publishing_timer = None
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Timers destroyed.")

            # Destroy publishers
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Destroying publishers...")
            pubs = {'ocm': self._offboard_control_mode_pub, 'traj': self._trajectory_setpoint_pub, 'cmd': self._vehicle_command_pub}
            for name, pub in pubs.items():
                 if pub:
                     if self._debug_logging: self.get_logger().debug(f"_cleanup_resources: Destroying publisher {name}")
                     self.destroy_publisher(pub)
            self._offboard_control_mode_pub = self._trajectory_setpoint_pub = self._vehicle_command_pub = None
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Publishers destroyed.")


             # Destroy subscribers
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Destroying subscribers...")
            subs = {'status': self._vehicle_status_sub, 'lpos': self._vehicle_local_position_sub}
            for name, sub in subs.items():
                 if sub:
                      if self._debug_logging: self.get_logger().debug(f"_cleanup_resources: Destroying subscriber {name}")
                      self.destroy_subscription(sub)
            self._vehicle_status_sub = self._vehicle_local_position_sub = None
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Subscribers destroyed.")


            self.get_logger().info("Resource cleanup helper finished.")
            if self._debug_logging: self.get_logger().debug("_cleanup_resources: Finished successfully.")


        except Exception as e:
            self.get_logger().error(f"Exception during _cleanup_resources: {e}")
            self.get_logger().error(traceback.format_exc())
            if self._debug_logging: self.get_logger().debug(f"_cleanup_resources: Exception occurred: {e}")
            success = False
        finally:
             # Reset state variables (already done in cleanup caller, but good practice here too)
             self._vehicle_status = None
             self._vehicle_local_position = None
             self._is_armed = False
             self._is_offboard = False
             self._is_on_ground_approx = True
             self._desired_setpoint = None
             self._active_goal_handle = None
             if self._debug_logging: self.get_logger().debug("_cleanup_resources: State variables reset in finally block.")
        return success

    # === PX4 Communication Callbacks ===

    def _vehicle_status_callback(self, msg: VehicleStatus):
        """Stores status and updates internal flags and state machine."""
        if self._debug_logging:
             # Log more detailed status, throttled
             self.get_logger().debug(f"Status Received: NAV={msg.nav_state}, ARM={msg.arming_state}, HIL={msg.hil_state}, Failsafe={msg.failsafe}, SystemID={msg.system_id}, CompID={msg.component_id}", throttle_duration_sec=2.0)
        self._vehicle_status = msg
        new_armed_state = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        new_offboard_state = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        # --- On-Ground Estimation Logic (Updated) ---
        # Heuristic based ONLY on arming_state since landed_state is missing
        new_on_ground_state = not new_armed_state # Assume ON_GROUND if DISARMED, IN_AIR if ARMED
        # Warning logged once if needed (moved the check to where it's used)
        # --- End of Updated On-Ground Logic ---


        # Update internal state machine based on changes (use lock for safety)
        with self._state_lock:
            current_internal_state = self._internal_state # Get state inside lock
            if self._debug_logging:
                self.get_logger().debug(f"Status Callback: Current internal state = {current_internal_state.name}", throttle_duration_sec=2.0)

            # --- Update primary flags ---
            armed_changed = new_armed_state != self._is_armed
            offboard_changed = new_offboard_state != self._is_offboard
            ground_changed = new_on_ground_state != self._is_on_ground_approx

            if armed_changed:
                if self._debug_logging: self.get_logger().debug(f"Status Callback: Armed state change detected: {self._is_armed} -> {new_armed_state}")
                self.get_logger().info(f"Arming state changed: {self._is_armed} -> {new_armed_state}")
                self._is_armed = new_armed_state

            if offboard_changed:
                if self._debug_logging: self.get_logger().debug(f"Status Callback: Offboard state change detected: {self._is_offboard} -> {new_offboard_state}")
                self.get_logger().info(f"Offboard mode changed: {self._is_offboard} -> {new_offboard_state}")
                self._is_offboard = new_offboard_state
                # If we exited offboard unexpectedly while trying to execute/hover/arming
                if not new_offboard_state and current_internal_state in [InternalState.IDLE_HOVERING, InternalState.EXECUTING_ACTION, InternalState.ARMING]:
                     self.get_logger().error("Exited OFFBOARD mode unexpectedly! Transitioning to IDLE_HOVERING (will attempt hold).")
                     # Force state change, hold_position will be commanded by timer default
                     self._internal_state = InternalState.IDLE_HOVERING # Set directly inside lock
                     self._active_goal_handle = None # Clear active goal
                     if self._debug_logging: self.get_logger().debug("Status Callback: Exited offboard unexpectedly, forced state to IDLE_HOVERING, cleared goal.")
                     # Hold command is implicitly set by the state transition logic below or timer

            if ground_changed:
                 # Log warning only once if needed
                 if not hasattr(msg, 'landed_state') and not hasattr(self, '_warned_missing_landed_state'):
                    self.get_logger().warn("VehicleStatus message missing 'landed_state'. Estimating on_ground based SOLELY on arming state (DISARMED=GROUND, ARMED=AIR). This is less reliable.")
                    self._warned_missing_landed_state = True # Prevent repeated warnings
                    if self._debug_logging: self.get_logger().debug("Status Callback: Logged warning for missing 'landed_state'.")

                 if self._debug_logging: self.get_logger().debug(f"Status Callback: On-ground state change detected (estimated): {self._is_on_ground_approx} -> {new_on_ground_state}")
                 self.get_logger().info(f"On-ground state changed (estimated): {self._is_on_ground_approx} -> {new_on_ground_state}")
                 self._is_on_ground_approx = new_on_ground_state


            # --- Update state machine based on combined flags (only if IDLE or state changed) ---
            state_changed_by_logic = False
            if current_internal_state in [InternalState.IDLE_ON_GROUND, InternalState.IDLE_HOVERING] or armed_changed or ground_changed:
                if self._debug_logging: self.get_logger().debug(f"Status Callback: Evaluating state machine update. Current={current_internal_state.name}, Armed={self._is_armed}, Ground={self._is_on_ground_approx}", throttle_duration_sec=1.0)

                # Transition from GROUND -> HOVERING
                if self._is_armed and not self._is_on_ground_approx and current_internal_state == InternalState.IDLE_ON_GROUND:
                    self.get_logger().info("State change detected: Armed and In-Air -> IDLE_HOVERING")
                    if self._debug_logging: self.get_logger().debug("Status Callback: Condition met for GROUND -> HOVERING transition.")
                    self._internal_state = InternalState.IDLE_HOVERING
                    self._update_desired_setpoint_to_hold() # Command hold in new state
                    state_changed_by_logic = True

                # Transition from HOVERING -> GROUND (Normal Disarm/Land)
                elif not self._is_armed and self._is_on_ground_approx and current_internal_state == InternalState.IDLE_HOVERING:
                    self.get_logger().info("State change detected: Disarmed and On-Ground -> IDLE_ON_GROUND")
                    if self._debug_logging: self.get_logger().debug("Status Callback: Condition met for HOVERING -> GROUND transition.")
                    self._internal_state = InternalState.IDLE_ON_GROUND
                    self._update_desired_setpoint_to_hold() # Command hold (at ground pos)
                    state_changed_by_logic = True

                # Transition from HOVERING -> GROUND (Unexpected Disarm while hovering)
                elif not self._is_armed and current_internal_state == InternalState.IDLE_HOVERING:
                     # This condition might overlap with the previous if ground state lags arm state.
                     # Prioritize the Disarmed & On-Ground case if both are true.
                     # This case handles disarm command received *before* ground contact is detected.
                     if not state_changed_by_logic: # Avoid double transition if already handled
                        self.get_logger().warn("Disarmed while IDLE_HOVERING. Transitioning to IDLE_ON_GROUND.")
                        if self._debug_logging: self.get_logger().debug("Status Callback: Condition met for HOVERING -> GROUND transition (Disarm while Hovering).")
                        self._internal_state = InternalState.IDLE_ON_GROUND
                        self._update_desired_setpoint_to_hold() # Command hold at last known pos
                        state_changed_by_logic = True
                # else: # No state transition based on logic
                #     if self._debug_logging: self.get_logger().debug("Status Callback: No state machine transition conditions met.", throttle_duration_sec=5.0)

            # Log final state after potential updates within the callback
            # if self._debug_logging and (armed_changed or offboard_changed or ground_changed or state_changed_by_logic):
            #     self.get_logger().debug(f"Status Callback Exit: Armed={self._is_armed}, Offboard={self._is_offboard}, Ground={self._is_on_ground_approx}, State={self._internal_state.name}")


    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Stores the latest vehicle local position."""
        if self._debug_logging:
             # Make this slightly more frequent for debugging motion
             self.get_logger().debug(f"Local Pos Rcvd: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}, VX={msg.vx:.2f}, VY={msg.vy:.2f}, VZ={msg.vz:.2f}, H={msg.heading:.2f}, DistBottom={msg.dist_bottom if msg.dist_bottom_valid else float('nan')}", throttle_duration_sec=1.0)
        self._vehicle_local_position = msg

    def _publish_offboard_control_mode_heartbeat(self):
        """Publishes the OffboardControlMode message periodically."""
        if self._offboard_control_mode_pub is None:
            if self._debug_logging: self.get_logger().debug("Heartbeat Timer: Publisher not available, skipping.", throttle_duration_sec=5.0)
            return
        msg = OffboardControlMode()
        # Set flags based on what handlers might command. Default to position.
        # TODO: Adapt these flags if handlers use other modes (velocity, attitude)
        msg.position = True
        msg.velocity = False # Set true if handlers use velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        try:
            if self._debug_logging:
                 # Log the message content, but maybe less frequently than heartbeat rate
                 self.get_logger().debug(f"Heartbeat Pub: Pos={msg.position}, Vel={msg.velocity}, Acc={msg.acceleration}, Att={msg.attitude}, Rate={msg.body_rate}", throttle_duration_sec=1.0)
            self._offboard_control_mode_pub.publish(msg)
        except Exception as e:
             self.get_logger().error(f"Failed to publish offboard control mode: {e}", throttle_duration_sec=5)
             if self._debug_logging: self.get_logger().debug(f"Heartbeat Timer: Exception during publish: {e}", throttle_duration_sec=5.0)


    def _publish_current_setpoint(self):
        """
        Periodically publishes the `self._desired_setpoint`.
        Defaults to holding current position if no desired setpoint is set.
        This runs continuously when the node is ACTIVE.
        """
        if self._trajectory_setpoint_pub is None:
            if self._debug_logging: self.get_logger().debug("Setpoint Timer: Publisher not available, skipping.", throttle_duration_sec=5.0)
            return

        publish_setpoint: Optional[TrajectorySetpoint] = None
        needs_hold_update = False
        # Access shared state within lock for consistency
        with self._state_lock:
            # Check if we have a valid desired setpoint
            if self._desired_setpoint is not None:
                publish_setpoint = self._desired_setpoint
                # Create a copy to avoid modifying the shared state outside lock if needed later, though we just read here.
                # publish_setpoint = TrajectorySetpoint(**vars(self._desired_setpoint))
            else:
                # No desired setpoint, need to command hold
                needs_hold_update = True

        if self._debug_logging and needs_hold_update:
             self.get_logger().debug("Setpoint Timer: No desired setpoint found, attempting to command HOLD.", throttle_duration_sec=2.0)

        # If no desired setpoint, default to HOLD (update happens outside primary lock to avoid holding lock during potential LPOS access)
        if needs_hold_update:
             hold_success = self._update_desired_setpoint_to_hold() # Tries to update self._desired_setpoint
             if hold_success:
                 with self._state_lock: # Re-acquire lock to read the potentially updated value
                      publish_setpoint = self._desired_setpoint
                 if self._debug_logging: self.get_logger().debug("Setpoint Timer: Successfully updated desired setpoint to HOLD.", throttle_duration_sec=2.0)
             else:
                  self.get_logger().warn("No desired setpoint and cannot get current position. Publishing NaN setpoint.", throttle_duration_sec=5)
                  if self._debug_logging: self.get_logger().debug("Setpoint Timer: Failed to update HOLD, creating NaN setpoint.", throttle_duration_sec=2.0)
                  # Publish NaNs as a fallback if position is unknown
                  nan_setpoint = TrajectorySetpoint()
                  nan_setpoint.position = [float('nan')] * 3
                  nan_setpoint.yaw = float('nan')
                  # Set others to NaN as well for clarity
                  nan_setpoint.velocity = [float('nan')] * 3
                  nan_setpoint.acceleration = [float('nan')] * 3
                  nan_setpoint.jerk = [float('nan')] * 3
                  nan_setpoint.yawspeed = float('nan')
                  publish_setpoint = nan_setpoint

        # Publish the determined setpoint
        if publish_setpoint:
            publish_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            try:
                # Use the previously acquired/created setpoint message
                self._trajectory_setpoint_pub.publish(publish_setpoint)
                if self._debug_logging:
                     pos = publish_setpoint.position
                     yaw = publish_setpoint.yaw
                     # Log published setpoint values, throttled
                     self.get_logger().debug(f"Setpoint Pub: N={pos[0]:.2f}, E={pos[1]:.2f}, D={pos[2]:.2f}, Yaw={yaw:.2f}", throttle_duration_sec=0.5) # Log more frequently
            except Exception as e:
                 self.get_logger().error(f"Failed to publish trajectory setpoint: {e}", throttle_duration_sec=5)
                 if self._debug_logging: self.get_logger().debug(f"Setpoint Timer: Exception during publish: {e}", throttle_duration_sec=5.0)

        # else: # Should not happen due to NaN fallback
        #    if self._debug_logging: self.get_logger().debug("Setpoint Timer: No setpoint determined to publish.", throttle_duration_sec=5.0)


    # === Public Methods for Action Handlers ===

    def is_armed(self) -> bool:
        """Returns the last known armed state."""
        # Ensure status is recent? Add timestamp check if needed.
        if self._debug_logging: self.get_logger().debug(f"is_armed() returning: {self._is_armed}", throttle_duration_sec=5.0)
        return self._is_armed

    def is_on_ground(self) -> bool:
        """Returns the estimated on-ground state."""
        # if self._debug_logging: self.get_logger().debug(f"is_on_ground() returning: {self._is_on_ground_approx}", throttle_duration_sec=5.0)
        return self._is_on_ground_approx

    def get_current_local_position(self) -> Optional[VehicleLocalPosition]:
        """Returns the last known VehicleLocalPosition message."""
        # Ensure position is recent? Add timestamp check if needed.
        # e.g., if self._vehicle_local_position and (self.get_clock().now().nanoseconds / 1000 - self._vehicle_local_position.timestamp) < POSITION_TIMEOUT_US:
        # return self._vehicle_local_position else return None
        if self._debug_logging:
            pos_time = self._vehicle_local_position.timestamp if self._vehicle_local_position else 0
            self.get_logger().debug(f"get_current_local_position() returning: {'Pose object' if self._vehicle_local_position else 'None'} (timestamp: {pos_time})", throttle_duration_sec=5.0)
        return self._vehicle_local_position

    def get_vehicle_status(self) -> Optional[VehicleStatus]:
         """Returns the last known VehicleStatus message."""
         # if self._debug_logging:
         #     status_time = self._vehicle_status.timestamp if self._vehicle_status else 0
         #     self.get_logger().debug(f"get_vehicle_status() returning: {'Status object' if self._vehicle_status else 'None'} (timestamp: {status_time})", throttle_duration_sec=5.0)
         return self._vehicle_status

    def set_target_setpoint(self, x: float = float('nan'), y: float = float('nan'), z: float = float('nan'), yaw: float = float('nan')):
        """
        Called by handlers to update the desired setpoint state.
        The setpoint timer will periodically publish this. Thread-safe.
        NaN values mean the controller should ignore that field (PX4 specific interpretation).
        """
        if self._debug_logging: self.get_logger().debug(f"set_target_setpoint called: N={x:.2f}, E={y:.2f}, D={z:.2f}, Yaw={yaw:.2f}")
        with self._state_lock: # Protect access to shared _desired_setpoint
            # Create or update the TrajectorySetpoint message safely
            if self._desired_setpoint is None:
                if self._debug_logging: self.get_logger().debug("set_target_setpoint: _desired_setpoint was None, creating new TrajectorySetpoint.")
                self._desired_setpoint = TrajectorySetpoint(
                     # Initialize all fields to NaN by default for clarity
                     position = [float('nan')] * 3,
                     velocity = [float('nan')] * 3,
                     acceleration = [float('nan')] * 3,
                     jerk = [float('nan')] * 3,
                     yaw = float('nan'),
                     yawspeed = float('nan')
                )
            # else:
            #    if self._debug_logging: self.get_logger().debug("set_target_setpoint: Updating existing _desired_setpoint.")


            # Update only the relevant fields for position control
            # Use float() to ensure conversion if needed, though inputs are typed
            self._desired_setpoint.position = [float(x), float(y), float(z)]
            self._desired_setpoint.yaw = float(yaw)
            # Ensure velocity/accel/rates are NaN if using pure position control via this method
            # If a handler needs mixed control, it should create/modify the setpoint directly
            self._desired_setpoint.velocity = [float('nan')] * 3
            self._desired_setpoint.acceleration = [float('nan')] * 3
            self._desired_setpoint.jerk = [float('nan')] * 3
            self._desired_setpoint.yawspeed = float('nan')
            if self._debug_logging:
                pos = self._desired_setpoint.position
                h = self._desired_setpoint.yaw
                self.get_logger().debug(f"set_target_setpoint: _desired_setpoint updated: N={pos[0]:.2f}, E={pos[1]:.2f}, D={pos[2]:.2f}, Yaw={h:.2f}")


    def hold_position(self) -> bool:
        """
        Updates the desired setpoint state to hold the current position.
        Returns True if successful in setting the state, False otherwise. Thread-safe.
        """
        self.get_logger().debug("hold_position() called. Attempting to update desired setpoint to current pose.")
        # Lock is acquired inside the helper
        result = self._update_desired_setpoint_to_hold()
        if self._debug_logging: self.get_logger().debug(f"hold_position() result: {result}")
        return result

    def _update_desired_setpoint_to_hold(self) -> bool:
         """Internal helper to update self._desired_setpoint to current pos. Thread-safe."""
         # This method might be called from different threads (state machine, timer, public call)
         # It needs to access _vehicle_local_position (updated by sub callback)
         # and update _desired_setpoint (read by timer callback / set by handlers)
         # -> Use lock for write operation on _desired_setpoint
         if self._debug_logging: self.get_logger().debug("_update_desired_setpoint_to_hold: Entering...")

         # Read LPOS - Reading doesn't strictly need lock if assignment is atomic, but safer to include
         # No, reading LPOS outside lock is fine. Lock needed only for WRITE to _desired_setpoint.
         current_pos = self._vehicle_local_position # Read volatile LPOS

         if current_pos is not None:
             # Check if position data is valid (using timestamp maybe?)
             # now_ns = self.get_clock().now().nanoseconds
             # pos_age_s = (now_ns / 1e9) - (current_pos.timestamp / 1e6)
             # if pos_age_s > 1.0: # Example: Position older than 1 second is stale
             #    self.get_logger().warn(f"Cannot update HOLD: Position data is stale ({pos_age_s:.2f}s old).")
             #    if self._debug_logging: self.get_logger().debug("_update_desired_setpoint_to_hold: Failed - Position data stale.")
             #    return False

             # Create or update the TrajectorySetpoint message for holding
             if self._desired_setpoint is None:
                  if self._debug_logging: self.get_logger().debug("_update_desired_setpoint_to_hold: _desired_setpoint was None, creating new.")
                  self._desired_setpoint = TrajectorySetpoint() # Create if doesn't exist

             # Update fields
             self._desired_setpoint.position = [current_pos.x, current_pos.y, current_pos.z]
             # Set velocities/accel/rates to NaN to ensure pure position control
             self._desired_setpoint.velocity = [float('nan')] * 3
             self._desired_setpoint.acceleration = [float('nan')] * 3
             self._desired_setpoint.jerk = [float('nan')] * 3
             # Hold current yaw if available and valid, otherwise let PX4 control it (NaN)
             # Note: VehicleLocalPosition heading is often not good for control unless aligned or magnetometer fused well.
             # Let's default to NaN yaw for HOLD unless explicitly needed.
             # self._desired_setpoint.yaw = current_pos.heading if current_pos.heading_good_for_control else float('nan')
             self._desired_setpoint.yaw = float('nan') # Default HOLD doesn't control yaw unless required by logic elsewhere
             self._desired_setpoint.yawspeed = float('nan')

             if self._debug_logging:
                  pos = self._desired_setpoint.position
                  h = self._desired_setpoint.yaw
                  self.get_logger().debug(f"_update_desired_setpoint_to_hold: SUCCESS - Setpoint updated to N={pos[0]:.2f}, E={pos[1]:.2f}, D={pos[2]:.2f}, Yaw={h:.2f}")
             return True
         else:
             self.get_logger().warning("Cannot update desired setpoint to Hold: Current local position unknown.", throttle_duration_sec=2.0)
             if self._debug_logging: self.get_logger().debug("_update_desired_setpoint_to_hold: FAILED - Current local position is None.")
             # Do NOT clear desired_setpoint here, as the timer will default to NaN anyway if it stays None.
             # Clearing it might overwrite a valid setpoint set by a handler just before LPOS was lost.
             return False


    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0, param4: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0) -> bool:
        """Helper to publish VehicleCommand. Assumes publisher exists."""
        if self._vehicle_command_pub is None:
            self.get_logger().error(f"Cannot publish VehicleCommand {command}: Publisher not available.")
            if self._debug_logging: self.get_logger().debug(f"_publish_vehicle_command: FAILED - Publisher missing for command {command}.")
            return False

        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5) # Typically X coordinate
        msg.param6 = float(param6) # Typically Y coordinate
        msg.param7 = float(param7) # Typically Z coordinate
        msg.command = command
        msg.target_system = self._target_system_id
        msg.target_component = 1 # Autopilot (usually 1)
        msg.source_system = 255 # GCS or Companion computer ID (use a specific one if assigned)
        msg.source_component = 1 # Or MAV_COMP_ID_ONBOARD_COMPUTERx (e.g., 191-197)
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self._debug_logging:
             self.get_logger().debug(f"Publishing VehicleCommand: CMD={command}, P1={param1:.1f}, P2={param2:.1f}, P3={param3:.1f}, P4={param4:.1f}, P5(X)={param5:.2f}, P6(Y)={param6:.2f}, P7(Z)={param7:.2f}, Target={msg.target_system}/{msg.target_component}")

        try:
            self._vehicle_command_pub.publish(msg)
            self.get_logger().info(f"Published VehicleCommand: {command} (P1={param1:.1f}, P2={param2:.1f}...)") # Keep INFO for confirmation
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish vehicle command {command}: {e}", throttle_duration_sec=5)
            if self._debug_logging: self.get_logger().debug(f"_publish_vehicle_command: FAILED - Exception during publish for command {command}: {e}")
            return False

    def arm(self) -> bool:
        """Sends the ARM command."""
        self.get_logger().info("Sending ARM command...")
        if self._debug_logging: self.get_logger().debug("arm() called: Sending VEHICLE_CMD_COMPONENT_ARM_DISARM (param1=1.0)")
        # Optionally check state before sending?
        current_state = self.get_internal_state()
        if current_state != InternalState.IDLE_ON_GROUND:
            self.get_logger().warn(f"Arm command sent while internal state is {current_state.name} (Expected IDLE_ON_GROUND).")
            if self._debug_logging: self.get_logger().debug(f"arm(): Warning - State is {current_state.name}")
        # VEHICLE_CMD_COMPONENT_ARM_DISARM: param1 = 1 to arm, 0 to disarm
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self) -> bool:
        """Sends the DISARM command."""
        self.get_logger().info("Sending DISARM command...")
        if self._debug_logging: self.get_logger().debug("disarm() called: Sending VEHICLE_CMD_COMPONENT_ARM_DISARM (param1=0.0)")
        # VEHICLE_CMD_COMPONENT_ARM_DISARM: param1 = 1 to arm, 0 to disarm
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def land(self) -> bool:
        """Sends the LAND command (sets mode to LAND)."""
        self.get_logger().info("Sending LAND command (changing mode to NAV_LAND)...")
        if self._debug_logging: self.get_logger().debug("land() called: Sending VEHICLE_CMD_NAV_LAND.")
        # Optionally update internal state here? Or let status callback handle it?
        # self.set_internal_state(InternalState.LANDING_AUTO) # Might be better handled by status callback seeing NAV_LAND mode
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def engage_offboard_mode(self) -> bool:
        """Sends command to switch to Offboard flight mode."""
        # PX4 requires continuous setpoints BEFORE this command is accepted.
        # Ensure setpoint timer is running and sending valid data. Check if node is active?
        if self._setpoint_publishing_timer is None or self._setpoint_publishing_timer.is_canceled():
            self.get_logger().warn("engage_offboard_mode() called but setpoint timer is not active. Command will likely fail.")
            if self._debug_logging: self.get_logger().debug("engage_offboard_mode(): Warning - Setpoint timer not active.")


        self.get_logger().info("Sending SET_MODE=OFFBOARD command...")
        if self._debug_logging: self.get_logger().debug("engage_offboard_mode() called: Sending VEHICLE_CMD_DO_SET_MODE (param1=1.0, param2=6.0)")
        # VEHICLE_CMD_DO_SET_MODE: param1 = base_mode (1 for custom mode), param2 = custom_mode (6 for Offboard)
        return self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)


# === Main Execution Logic ===
def main(args: Optional[list[str]] = None):
     """ Main function to run the node standalone. """
     print('Starting Core Drone Controller Node...')
     node: Optional[CoreDroneController] = None
     executor: Optional[MultiThreadedExecutor] = None
     debug_logging_enabled_early = '--ros-args' in (args or []) and '--log-level' in (args or []) and 'debug' in (args or []) # Basic check

     try:
         rclpy.init(args=args)
         node = CoreDroneController()
         # Early check if debug might be enabled for main logic logging
         # Node's internal _debug_logging isn't set until configure, so we use the args check
         if debug_logging_enabled_early: node.get_logger().debug("[Main] Node instance created.")

         executor = MultiThreadedExecutor()
         executor.add_node(node)
         executor.spin()
         # rclpy.spin(node)

     except KeyboardInterrupt:
         if node: node.get_logger().info('Keyboard interrupt, shutting down...')
         else: print('Keyboard interrupt, shutting down...')
         if node and debug_logging_enabled_early: node.get_logger().debug("[Main] KeyboardInterrupt caught.")
     except Exception as e:
          if node: node.get_logger().error(f"Exception in main spin: {e}")
          else: print(f"Exception before node init: {e}")
          traceback.print_exc()
          if node and debug_logging_enabled_early: node.get_logger().debug(f"[Main] Exception caught: {e}")
     finally:
         # Orderly shutdown
         if node and executor and rclpy.ok():
             node.get_logger().info('Initiating node shutdown sequence...')
             if node._debug_logging: node.get_logger().debug("[Main] Starting final shutdown sequence...") # Use node's logger if available now
             # Simplified shutdown: just trigger transitions downwards
             # Checking state during shutdown after errors can be unreliable
             try:
                 if node._debug_logging: node.get_logger().debug('[Main] Attempting Deactivate trigger...')
                 node.trigger_deactivate()
                 executor.spin_once(timeout_sec=0.5) # Shorter timeout during shutdown
             except Exception as e:
                 node.get_logger().error(f"Error during deactivate trigger: {e}")
             try:
                  if node._debug_logging: node.get_logger().debug('[Main] Attempting Cleanup trigger...')
                  node.trigger_cleanup()
                  executor.spin_once(timeout_sec=0.5)
             except Exception as e:
                  node.get_logger().error(f"Error during cleanup trigger: {e}")
             try:
                  if node._debug_logging: node.get_logger().debug('[Main] Attempting Shutdown trigger...')
                  node.trigger_shutdown()
                  executor.spin_once(timeout_sec=0.5)
             except Exception as e:
                  node.get_logger().error(f"Error during shutdown trigger: {e}")


             node.get_logger().info('Shutting down executor...')
             if node._debug_logging: node.get_logger().debug("[Main] Shutting down executor.")
             executor.shutdown()

         if rclpy.ok():
             print('Shutting down RCLPY...')
             # No logger available here typically
             rclpy.shutdown()
         print('Shutdown complete.')


if __name__ == '__main__':
     main()
