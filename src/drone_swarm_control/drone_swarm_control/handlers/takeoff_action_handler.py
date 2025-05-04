import rclpy
import time
import traceback
import math
from rclpy.action.server import GoalResponse, CancelResponse, ServerGoalHandle
from rclpy.lifecycle import LifecycleState, LifecycleNode
from rclpy.duration import Duration
from typing import Type, Any, Optional

# Import the base class (ensure the relative path is correct)
# If this file is in drone_swarm_control/handlers/
# and base is in drone_swarm_control/handlers/
from .core_action_handler_base import ActionHandlerBase

# Import the specific action type
# Replace 'swarm_interfaces' with your actual interface package name
from swarm_interfaces.action import Takeoff

# Import the InternalState enum from the node definition
# This creates a dependency, consider moving InternalState to a common location if preferred
try:
    # Adjust relative path if needed
    from drone_swarm_control.utils.types import InternalState
except ImportError:
    print("Could not import InternalState from core_drone_controller_node. Using dummy values.")
    # Define dummy enum if import fails to prevent NameError
    from enum import Enum, auto
    class InternalState(Enum):
        UNKNOWN = auto()
        IDLE_ON_GROUND = auto()
        ARMING = auto()
        IDLE_HOVERING = auto()
        EXECUTING_ACTION = auto()


# Constants
DEFAULT_TAKEOFF_VELOCITY = 0.5  # m/s upwards (Not directly used if using position setpoints)
ALTITUDE_TOLERANCE = 0.25       # meters - How close to target height is considered 'reached'
STABILIZATION_TIME = 1.0        # seconds - Time to hover after reaching altitude
MAX_TAKEOFF_HEIGHT = 20.0       # meters - Safety limit
MIN_TAKEOFF_HEIGHT = 0.5        # meters - Safety limit
ARM_TIMEOUT_SEC = 10.0          # Timeout for waiting for arm confirmation


class TakeoffActionHandler(ActionHandlerBase):
    """
    Action Handler for the Takeoff action.
    Manages arming the drone and ascending vertically to a specified height.
    Interacts with the CoreDroneController's internal state machine.
    """

    def __init__(self, parent_node: LifecycleNode, logger: Any):
        """
        Initialize the TakeoffActionHandler.

        Args:
            parent_node: The CoreDroneController node.
            logger: The logger instance from the parent node.
        """
        super().__init__(
            parent_node=parent_node,
            logger=logger,
            action_name='takeoff', # Action server name
            action_type=Takeoff    # Action type definition
        )
        self._takeoff_start_z: Optional[float] = None # Stores the initial Z position (NED) at takeoff start

    # --- Implementation of Abstract Methods ---

    def goal_callback(self, goal_request: Takeoff.Goal) -> GoalResponse:
        """
        Accept or reject a Takeoff goal request.
        Checks parent node state, internal state, drone status, and goal parameters.
        """
        self.logger.info(f"[{self.__class__.__name__}] Received takeoff goal request to {goal_request.target_height:.2f}m.")

        # 1. Check if Action Server is ready
        if not self.is_started or self._action_server is None:
            self.logger.error("Rejecting goal: Takeoff action server not started.")
            return GoalResponse.REJECT


        # 3. Check parent node's Internal State
        current_internal_state = self.parent_node.get_internal_state()
        if current_internal_state != InternalState.IDLE_ON_GROUND:
            self.logger.warn(f"Rejecting goal: Invalid internal state ({current_internal_state.name}) for Takeoff. Expected IDLE_ON_GROUND.")
            return GoalResponse.REJECT

        # 4. Check drone status via parent node methods
        try:
            is_armed = self.parent_node.is_armed()
            is_on_ground = self.parent_node.is_on_ground()
            local_pos = self.parent_node.get_current_local_position()

            if is_armed: # Should not be armed if state is IDLE_ON_GROUND, but double-check
                 self.logger.warn("Rejecting goal: Drone is already armed (inconsistent state?).")
                 return GoalResponse.REJECT
            if not is_on_ground:
                 self.logger.warn("Rejecting goal: Drone is not estimated to be on ground.")
                 return GoalResponse.REJECT
            if local_pos is None:
                self.logger.warn("Rejecting goal: Current local position unknown.")
                return GoalResponse.REJECT

        except AttributeError as e:
             self.logger.error(f"Rejecting goal: Parent node missing required status method: {e}")
             return GoalResponse.REJECT
        except Exception as e:
            self.logger.error(f"Rejecting goal: Error checking drone status: {e}")
            return GoalResponse.REJECT

        # 5. Validate goal parameters
        if not (MIN_TAKEOFF_HEIGHT <= goal_request.target_height <= MAX_TAKEOFF_HEIGHT):
            self.logger.warn(f"Rejecting goal: Target height {goal_request.target_height:.2f}m is outside safe limits [{MIN_TAKEOFF_HEIGHT}m, {MAX_TAKEOFF_HEIGHT}m].")
            return GoalResponse.REJECT

        # If all checks pass, accept the goal
        self.logger.info(f"Accepting takeoff goal to {goal_request.target_height:.2f}m.")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """ Start the asynchronous execution and update parent state. """
        self.logger.info(f"[{self.__class__.__name__}] Goal accepted: {goal_handle.goal_id}. Starting execution...")
        # Set parent state to ARMING (or EXECUTING_ACTION) before starting execution
        # Pass the goal handle so the parent knows which goal is active
        self.parent_node.set_internal_state(InternalState.ARMING, goal_handle)
        goal_handle.execute() # Start the execute_callback

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """ Handle cancellation requests. """
        self.logger.info(f"[{self.__class__.__name__}] Received cancel request for goal {goal_handle.goal_id}.")
        # Check if the goal being cancelled is the currently active one
        active_goal = self.parent_node.get_active_goal_handle()
        if active_goal and active_goal.goal_id == goal_handle.goal_id:
            self.logger.info("Accepting cancellation request for the active goal.")
            return CancelResponse.ACCEPT
        else:
            self.logger.warn(f"Rejecting cancel request: Goal {goal_handle.goal_id} is not the active goal.")
            return CancelResponse.REJECT


    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Takeoff.Result:
        """
        Executes the takeoff action: arm, ascend, monitor, stabilize.
        Updates parent state machine on completion/error/cancel.
        """
        goal_id_str = goal_handle.goal_id # Use hex for shorter log messages
        self.logger.info(f"[{self.__class__.__name__}] Executing goal {goal_id_str}...")

        request: Takeoff.Goal = goal_handle.request
        result = Takeoff.Result()
        feedback = Takeoff.Feedback()

        # --- Pre-flight Setup ---
        try:
            # 1. Get initial position (NED Z-coordinate)
            initial_pos = self.parent_node.get_current_local_position()
            if initial_pos is None:
                raise RuntimeError("Cannot get initial position for takeoff.")
            self._takeoff_start_z = initial_pos.z # Store starting altitude (NED)
            initial_x = initial_pos.x
            initial_y = initial_pos.y
            self.logger.info(f"Takeoff starting from X={initial_x:.2f}, Y={initial_y:.2f}, Z={self._takeoff_start_z:.2f}m (NED)")

            # 2. Determine target NED Z-coordinate
            # Target height is positive UP, NED Z is positive DOWN
            target_z_ned = self._takeoff_start_z - request.target_height

            # 3. Arm the drone via parent node
            self.logger.info(f"[{goal_id_str}] Requesting ARM...")
            if not self.parent_node.arm(): # Assumed method returns success/failure
                raise RuntimeError("Failed to send arm command via parent.")

            # 4. Wait for armed confirmation (with timeout)
            arm_start_time = self.parent_node.get_clock().now()
            while not self.parent_node.is_armed():
                if (self.parent_node.get_clock().now() - arm_start_time).nanoseconds / (2*1e9) > ARM_TIMEOUT_SEC:
                     raise TimeoutError(f"Timed out ({ARM_TIMEOUT_SEC}s) waiting for drone to arm.")
                # Check for cancellation while waiting
                if goal_handle.is_cancel_requested:
                    self.logger.info(f"[{goal_id_str}] Canceled while waiting for arm.")
                    # No motion started, just transition state back
                    self.parent_node.set_internal_state(InternalState.IDLE_ON_GROUND)
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Takeoff canceled while waiting for arm."
                    result.final_height = 0.0
                    return result
                self.parent_node.get_clock().sleep_for(Duration(seconds=0.1))
            self.logger.info(f"[{goal_id_str}] Drone is ARMED.")
            # Update state machine now that arming is confirmed
            self.parent_node.set_internal_state(InternalState.EXECUTING_ACTION, goal_handle)


            # 5. Engage Offboard Mode (Important!)
            # Needs to happen after arming and before significant motion command
            self.logger.info(f"[{goal_id_str}] Engaging Offboard mode...")
            if not self.parent_node.engage_offboard_mode():
                 # This might fail if setpoint stream wasn't active long enough beforehand
                 raise RuntimeError("Failed to send engage offboard mode command.")
            # Allow a short time for mode switch confirmation if needed, or proceed
            await self.parent_node.get_clock().sleep_for(Duration(seconds=0.2))
            # Add check for self.parent_node.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD ?


            # --- Ascent Loop ---
            self.logger.info(f"[{goal_id_str}] Starting ascent to target Z={target_z_ned:.2f}m (NED)...")
            while rclpy.ok():
                # ** Cancellation Check (Top of Loop) **
                if goal_handle.is_cancel_requested:
                    self.logger.info(f"[{goal_id_str}] Cancellation requested during ascent. Commanding hold.")
                    self.parent_node.hold_position() # Update desired setpoint FIRST
                    self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Then update state
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Takeoff canceled during ascent."
                    # Get final height for result
                    final_pos = self.parent_node.get_current_local_position()
                    result.final_height = abs(final_pos.z - self._takeoff_start_z) if final_pos and self._takeoff_start_z is not None else 0.0
                    return result

                # ** Get Current State **
                current_pos = self.parent_node.get_current_local_position()
                if current_pos is None:
                    raise RuntimeError("Lost position estimate during takeoff ascent.")

                current_z_ned = current_pos.z
                current_height_relative = abs(current_z_ned - self._takeoff_start_z) if self._takeoff_start_z is not None else 0.0

                # ** Publish Feedback **
                feedback.current_height = current_height_relative
                feedback.height_remaining = max(0.0, request.target_height - current_height_relative)
                goal_handle.publish_feedback(feedback)

                # ** Check if Target Altitude Reached **
                if abs(current_z_ned - target_z_ned) < ALTITUDE_TOLERANCE:
                    self.logger.info(f"[{goal_id_str}] Reached target altitude zone (Z_NED={current_z_ned:.2f}m).")
                    break # Exit ascent loop

                # ** Command Motion: Update Desired Setpoint **
                # Command hover at initial X, Y, but target Z
                self.parent_node.set_target_setpoint(
                    x=initial_x, # Hold horizontal position at takeoff point
                    y=initial_y,
                    z=target_z_ned,  # Command target altitude
                    yaw=float('nan') # Let PX4 manage yaw (or command initial yaw)
                )

                # ** Loop Sleep **
                await self.parent_node.get_clock().sleep_for(Duration(seconds=0.1)) # Match controller rate?

            # --- Post-Ascent Stabilization ---
            self.logger.info(f"[{goal_id_str}] Stabilizing at target altitude...")
            stabilization_start_time = self.parent_node.get_clock().now()
            while (self.parent_node.get_clock().now() - stabilization_start_time).nanoseconds / 1e9 < STABILIZATION_TIME:
                 # ** Cancellation Check (During Stabilization) **
                if goal_handle.is_cancel_requested:
                    self.logger.info(f"[{goal_id_str}] Cancellation requested during stabilization. Holding.")
                    self.parent_node.hold_position() # Update desired setpoint FIRST
                    self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Then update state
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Takeoff canceled during stabilization."
                    final_pos = self.parent_node.get_current_local_position()
                    result.final_height = abs(final_pos.z - self._takeoff_start_z) if final_pos and self._takeoff_start_z is not None else 0.0
                    return result

                # Keep commanding hold at the reached position
                final_pos = self.parent_node.get_current_local_position()
                if final_pos:
                    # Update desired setpoint to hold the current stabilized position
                    self.parent_node.set_target_setpoint(
                         x=final_pos.x,
                         y=final_pos.y,
                         z=final_pos.z, # Hold the actual Z reached
                         yaw=float('nan') # Or final_pos.heading
                    )
                else:
                     # If pos lost here, maybe just keep sending target Z? Less ideal.
                     self.parent_node.set_target_setpoint(x=initial_x, y=initial_y, z=target_z_ned, yaw=float('nan'))

                await self.parent_node.get_clock().sleep_for(Duration(seconds=0.1))

            # --- Success ---
            self.logger.info(f"[{goal_id_str}] Takeoff successful and stabilized.")
            # Set state back to idle *before* succeeding goal
            self.parent_node.set_internal_state(InternalState.IDLE_HOVERING)
            goal_handle.succeed()
            result.success = True
            final_pos = self.parent_node.get_current_local_position()
            result.final_height = abs(final_pos.z - self._takeoff_start_z) if final_pos and self._takeoff_start_z is not None else request.target_height
            result.message = f"Takeoff complete at {result.final_height:.2f}m"
            return result

        except Exception as e:
            self.logger.error(f"[{goal_id_str}] Exception during Takeoff execution: {e}")
            self.logger.error(traceback.format_exc())
            # Safety: Command hold position on error
            final_height_on_error = 0.0 # Default
            try:
                self.logger.info(f"[{goal_id_str}] Attempting to hold position after exception.")
                # Get height *before* trying to hold, in case hold fails
                final_pos = self.parent_node.get_current_local_position()
                if final_pos and self._takeoff_start_z is not None:
                     final_height_on_error = abs(final_pos.z - self._takeoff_start_z)

                self.parent_node.hold_position() # Update desired setpoint FIRST
            except Exception as hold_e:
                self.logger.error(f"[{goal_id_str}] Failed to command hold during exception handling: {hold_e}")
            finally:
                # Set state back even if hold failed
                self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Or a potential ERROR state?
                goal_handle.abort() # Mark goal as aborted
                result.success = False
                result.message = f"Error during takeoff: {str(e)}"
                result.final_height = final_height_on_error
                return result
        finally:
            # Reset handler-specific state if needed
            self._takeoff_start_z = None
            self.logger.debug(f"[{self.__class__.__name__}] execute_callback finished for goal {goal_id_str}")

