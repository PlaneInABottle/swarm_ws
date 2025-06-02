import rclpy
import time
import asyncio
import traceback
import math
import functools
from rclpy.action.server import GoalResponse, CancelResponse, ServerGoalHandle
from rclpy.lifecycle import LifecycleState, LifecycleNode
from rclpy.duration import Duration
from typing import Type, Any, Optional

from rclpy.task import Future

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
    from simple_controller.utils.types import InternalState
except ImportError:
    print(
        "Could not import InternalState from core_drone_controller_node. Using dummy values."
    )
    # Define dummy enum if import fails to prevent NameError
    from enum import Enum, auto

    class InternalState(Enum):
        UNKNOWN = auto()
        IDLE_ON_GROUND = auto()
        ARMING = auto()
        IDLE_HOVERING = auto()
        EXECUTING_ACTION = auto()


# Constants
DEFAULT_TAKEOFF_VELOCITY = (
    0.5  # m/s upwards (Not directly used if using position setpoints)
)
ALTITUDE_TOLERANCE = 0.25  # meters - How close to target height is considered 'reached'
STABILIZATION_TIME = 1.0  # seconds - Time to hover after reaching altitude
MAX_TAKEOFF_HEIGHT = 20.0  # meters - Safety limit
MIN_TAKEOFF_HEIGHT = 0.5  # meters - Safety limit
ARM_TIMEOUT_SEC = 10.0  # Timeout for waiting for arm confirmation


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
            action_name="takeoff",  # Action server name
            action_type=Takeoff,  # Action type definition
        )
        self._takeoff_start_z: Optional[float] = (
            None  # Stores the initial Z position (NED) at takeoff start
        )

    # --- Implementation of Abstract Methods ---

    def goal_callback(self, goal_request: Takeoff.Goal) -> GoalResponse:
        """
        Accept or reject a Takeoff goal request.
        Checks parent node state, internal state, drone status, and goal parameters.
        """
        self.logger.info(
            f"[{self.__class__.__name__}] Received takeoff goal request to {goal_request.target_height:.2f}m."
        )

        # 1. Check if Action Server is ready
        if not self.is_started or self._action_server is None:
            self.logger.error("Rejecting goal: Takeoff action server not started.")
            return GoalResponse.REJECT

        self.logger.info(
            f"Accepting takeoff goal to {goal_request.target_height:.2f}m."
        )
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Start the asynchronous execution and update parent state."""
        self.logger.info(
            f"[{self.__class__.__name__}] Goal accepted: {goal_handle.goal_id}. Starting overriden method execution..."
        )
        goal_handle.execute()  # Start the execute_callback

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancellation requests."""
        self.logger.info(
            f"[{self.__class__.__name__}] Received cancel request for goal {goal_handle.goal_id}."
        )
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Takeoff.Result:
        """
        Executes the takeoff action: arm, ascend, monitor, stabilize.
        Updates parent state machine on completion/error/cancel.
        """
        result = Takeoff.Result()
        feedback = Takeoff.Feedback()
        request: Takeoff.Goal = goal_handle.request

        self.execute_future = Future()

        # --- 1. Preparation and Sanity Checks ---
        current_pos = self.parent_node.get_current_local_position()
        if current_pos is None:
            result.success = False
            result.message = "Failed to get current position at takeoff start."
            self.logger.error(result.message)
            goal_handle.abort()
            # self.parent_node.set_internal_state(InternalState.IDLE_ON_GROUND) # Or appropriate error state
            return result

        self._takeoff_start_z = current_pos.z
        self._target_height_relative = request.target_height
        self._target_z_ned = self._takeoff_start_z - self._target_height_relative

        # --- 2. Engage Offboard and Arm ---
        try:
            self.logger.info("Engaging Offboard Mode...")
            if not await self.parent_node.engage_offboard_mode():
                raise RuntimeError("Failed to engage Offboard mode.")
            self.logger.info("Offboard mode engaged.")

            self.logger.info("Arming drone...")
            if not await self.parent_node.engage_arm_mode():
                raise RuntimeError("Failed to arm drone.")
            self.logger.info("Drone armed.")
            # self.parent_node.set_internal_state(InternalState.ARMING) # Or similar
        except RuntimeError as e:
            result.success = False
            result.message = str(e)
            self.logger.error(f"Takeoff pre-flight check failed: {e}")
            goal_handle.abort()
            # self.parent_node.set_internal_state(InternalState.IDLE_ON_GROUND) # Or error state
            return result

        # --- 3. Start Feedback Timer and Await Completion ---
        # Initialize feedback
        feedback.current_height = -self._takeoff_start_z  # Positive height above start (approx, if start_z is near 0 relative to some origin)
        feedback.height_remaining = self._target_height_relative

        partial_callback = functools.partial(
            self.execute_feedback_callback, feedback, goal_handle, request
        )
        self.execute_feedback_timer = self.parent_node.create_timer(
            self.execute_timer_period,
            partial_callback,
            callback_group=self._callback_group,  # Can be same or different CB group
        )

        action_outcome = await self.execute_future

        # Clean up
        if self.execute_feedback_timer:
            self.execute_feedback_timer.cancel()
        self.execute_feedback_timer = None
        self.execute_future = None
        self._takeoff_start_z = None
        self._target_height_relative = None
        self._target_z_ned = None
        self._action_start_time = None

        # --- 4. Process Result ---
        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Takeoff canceled by request."
            goal_handle.canceled()
            self.logger.info(result.message)
            # self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Or as appropriate
        elif action_outcome and action_outcome.get("success", False):
            result.success = True
            result.message = action_outcome.get("message", "Takeoff successful.")
            result.final_height = action_outcome.get("final_height", 0.0)
            goal_handle.succeed()
            self.logger.info(result.message)
            # self.parent_node.set_internal_state(InternalState.IDLE_HOVERING)
        else:
            result.success = False
            result.message = action_outcome.get("message", "Takeoff failed.")
            goal_handle.abort()
            self.logger.error(result.message)
            # self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Or appropriate error state

        return result

    async def execute_feedback_callback(self, feedback, goal_handle, request):
        if (
            not self.is_started
            or self._takeoff_start_z is None
            or self._target_height_relative is None
            or self._target_z_ned is None
        ):
            self.logger.warn("Takeoff not properly initialized. Skipping feedback.")
            return

        # --- 1. Handle Cancellation ---
        if goal_handle.is_cancel_requested:
            if self.execute_future and not self.execute_future.done():
                self.execute_future.set_result(
                    {"success": False, "message": "Takeoff canceled during execution."}
                )
            # Command drone to hold current position before exiting
            # self.parent_node.hold_current_position() # Implement this in parent
            return

        # --- 2. Get Current State ---
        current_position = self.parent_node.get_current_local_position()
        if current_position is None:
            self.logger.warn("Cannot get current position in feedback loop.")
            return

        current_z_ned = current_position.z

        # --- 3. Check Arm Status (important during ascent) ---
        if not self.parent_node.is_armed():
            self.logger.error("Drone became disarmed during takeoff!")
            if self.execute_future and not self.execute_future.done():
                self.execute_future.set_result(
                    {"success": False, "message": "Drone disarmed during takeoff."}
                )
            return

        # --- 4. Command Drone (Using Velocity or Position Control) ---
        # Determine the desired vertical velocity.
        # NED: positive velocity is downwards, negative velocity is upwards.
        desired_vertical_velocity_ned = -DEFAULT_TAKEOFF_VELOCITY  # Default upwards
        if request.takeoff_velocity > 0:  # User provided a positive "upwards" velocity
            desired_vertical_velocity_ned = -abs(request.takeoff_velocity)
            self.logger.debug(
                f"Using requested takeoff_velocity: {-desired_vertical_velocity_ned:.2f} m/s upwards."
            )
        else:
            self.logger.debug(
                f"Using default takeoff_velocity: {-desired_vertical_velocity_ned:.2f} m/s upwards."
            )

        self.parent_node.set_target_velocity_ned(
            0.0, 0.0, desired_vertical_velocity_ned, 0.0
        )

        # --- 5. Calculate Feedback (Consistent with Action Definition) ---

        # `current_height`: current height relative to takeoff point (positive upwards)
        # `self._takeoff_start_z` is NED (e.g., -0.1m)
        # `current_z_ned` is NED (e.g., -2.1m if 2m above start)
        # Height ascended = `self._takeoff_start_z` - `current_z_ned`
        # e.g., -0.1 - (-2.1) = 2.0m
        feedback.current_height = self._takeoff_start_z - current_z_ned

        # `height_remaining`: Estimated height remaining to target (positive)
        # `self._target_z_ned` is the absolute target Z in NED (e.g., _takeoff_start_z - target_height_relative)
        # `height_remaining_ned_difference = current_z_ned - self._target_z_ned`
        # e.g., current_z_ned = -2.1, target_z_ned = -5.1 (for a 5m takeoff from -0.1m)
        # remaining = -2.1 - (-5.1) = 3.0m
        height_remaining_ned_diff = current_z_ned - self._target_z_ned
        feedback.height_remaining = max(0.0, height_remaining_ned_diff)

        goal_handle.publish_feedback(feedback)

        # --- 6. Check for Completion ---
        # `height_remaining_ned_diff` is current_z - target_z.
        # If at target, this is 0. If below target (still ascending), it's positive.
        # If overshot (more negative Z than target), it's negative.
        if abs(height_remaining_ned_diff) < ALTITUDE_TOLERANCE:
            self.logger.info(
                f"Target altitude reached (Height remaining diff: {height_remaining_ned_diff:.2f}m, Tolerance: {ALTITUDE_TOLERANCE}m)."
            )
            # ... (Stabilization logic, sonra ekleriz ) ...
            self.parent_node.set_target_position_ned(0.0, 0.0, self._target_z_ned)

            if self.execute_future and not self.execute_future.done():
                # For the result, final_height is also relative to takeoff point
                final_height_relative = self._takeoff_start_z - current_z_ned
                self.execute_future.set_result(
                    {
                        "success": True,
                        "message": "Takeoff successful, altitude reached.",
                        "final_height": final_height_relative,
                    }
                )
