import rclpy
import time
import asyncio
import traceback
import math
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
        self.execute_future = Future()
        self.feedback_timer = None
        result = Takeoff.Result()
        feedback = Takeoff.Feedback()

        def feedback_callback():
            if self.is_started:
                result = Takeoff.Result()
                result.success = True
                result.message = f"Error during takeoff"

                result.final_height = 1.0
                feedback.current_height += 1.0
                feedback.height_remaining += 2.0
                self.logger.info("Publishing Feedback")
                goal_handle.publish_feedback(feedback)
                if feedback.current_height >= 50:
                    self.execute_future.set_result(None)
                    self.feedback_timer.cancel()

        self.feedback_timer_period = 0.5  # seconds (e.g., 2 Hz)
        self.feedback_timer = self.parent_node.create_timer(
            self.feedback_timer_period,
            feedback_callback,
            callback_group=self._callback_group,  # Can be same or different CB group
        )
        await self.execute_future

        # Make Drone Offboard
        self.logger.info(f"Engaging Offboard Mode")
        await self.parent_node.engage_offboard_mode()
        self.logger.info(f"Engaging Arm Mode")
        await self.parent_node.engage_arm_mode()
        # self.feedback_timer.reset()

        self.logger.info(f"Goal handler Request: {goal_handle.request}")
        request: Takeoff.Goal = goal_handle.request

        self.logger.info(f"Normal Requst  {request}")

        self.logger.info("Setting Result")
        result.success = True
        result.message = f"Error during takeoff"
        self.logger.info("Setting ResultV2")
        result.final_height = 1.0

        # ** Publish Feedback **
        # Be carefull about integer or float. 0.0 is float
        goal_handle.succeed()

        self.logger.info("Returning result")
        return result
