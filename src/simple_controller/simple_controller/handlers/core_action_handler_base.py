# drone_swarm_control/handlers/action_handler_base.py

import abc
import rclpy
import traceback
from typing import Type, Any, Optional

# Import specific rclpy types for hints
from rclpy.action.server import (
    ActionServer,
    GoalResponse,
    CancelResponse,
    ServerGoalHandle,
)
from rclpy.callback_groups import (
    CallbackGroup,
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.lifecycle import LifecycleNode
from rclpy.task import Future
from rclpy.timer import Timer


class ActionHandlerBase(abc.ABC):
    """
    Abstract Base Class for ROS 2 Action Handlers managed by the CoreDroneController.

    Each concrete handler derived from this class will manage a specific
    ROS 2 Action Server (e.g., Takeoff, Land, NavigateToPoint).
    """

    # Use 'Any' for the logger type hint as direct import failed
    def __init__(
        self,
        parent_node: LifecycleNode,
        logger: Any,
        action_name: str,
        action_type: Type,
    ):
        """
        Initializes the base action handler.

        Args:
            parent_node: The ROS 2 LifecycleNode (typically CoreDroneController) that owns this handler.
                         Provides access to shared resources and methods like hold_position(), clock, etc.
            logger: The logger instance obtained from the parent node (e.g., parent_node.get_logger()).
            action_name: The name of the action server (e.g., 'takeoff', 'land').
            action_type: The type of the action definition (e.g., Takeoff, Land from interfaces).
        """
        self._parent_node = parent_node
        self._logger = logger  # Store the passed logger instance
        self._action_name = action_name
        self._action_type = action_type
        self._action_server: Optional[ActionServer] = None
        self._callback_group = None
        self._is_started = False
        self.execute_future = None
        self.execute_feedback_timer: Optional[Timer] = None
        self.execute_timer_period = 1

    @property
    def logger(self) -> Any:  # Changed return type hint to Any
        """Provides read-only access to the logger instance."""
        return self._logger

    @property
    def parent_node(self) -> LifecycleNode:
        """Provides read-only access to the parent node."""
        return self._parent_node

    @property
    def is_started(self) -> bool:
        """Returns True if the action server has been successfully started."""
        return self._is_started

    def start(
        self,
        callback_group: Optional[CallbackGroup] = None,
    ) -> bool:
        """
        Creates and initializes the ROS 2 Action Server for this handler.
        Should only be called once, typically during the parent node's configuration phase.

        Returns:
            True if the server started successfully, False otherwise.
        """
        if self._action_server is not None:
            # Use the stored logger instance
            self.logger.warn(
                f"[{self.__class__.__name__}] Action server '{self._action_name}' already started."
            )
            return True  # Already started, consider it success

        # self.logger.debug(
        #     f"[{self.__class__.__name__}] Starting Action Server '{self._action_name}'..."
        # )
        try:
            full_action_name = f"{self.parent_node.get_name()}/{self._action_name}"

            # Change default callback groups
            self._callback_group = callback_group
            self._action_server = ActionServer(
                node=self.parent_node,
                action_type=self._action_type,
                action_name=full_action_name,
                execute_callback=self.execute_callback, # 3
                goal_callback=self.goal_callback,  # 1 validation methodumuz
                handle_accepted_callback=self.handle_accepted_callback,  # 2
                cancel_callback=self.cancel_callback, # optional
                callback_group=callback_group,
            )
            # self.logger.debug(
            #     f"[{self.__class__.__name__}] Action Server '{full_action_name}' started successfully."
            # )

            self._is_started = True
            return True
        except Exception as e:
            # Hnadle the other variables
            self.logger.error(
                f"[{self.__class__.__name__}] Failed to start Action Server '{full_action_name}': {e}"
            )
            self.logger.error(traceback.format_exc())
            self._action_server = None
            self._is_started = False
            return False

    def stop(self):
        """
        Cleans up the Action Server resources.
        Should be called during the parent node's cleanup or shutdown phase.
        Note: ActionServer doesn't have an explicit destroy method in typical use,
        relying on garbage collection when references are removed. Setting to None helps.
        """
        if self._action_server is not None:
            # self.logger.debug(
            #     f"[{self.__class__.__name__}] Stopping Action Server '{self._action_name}'..."
            # )
            # TODO: Check if active goals need explicit cancellation here during stop.
            # This might depend on the desired shutdown behavior.
            self._action_server = None  # Remove reference to allow garbage collection
            self._is_started = False
            # self.logger.debug(
            #     f"[{self.__class__.__name__}] Action Server '{self._action_name}' stopped."
            # )
        else:
            self.logger.debug(
                f"[{self.__class__.__name__}] Action server '{self._action_name}' was not running or already stopped."
            )

    # --- Abstract Methods to be Implemented by Concrete Handlers ---

    @abc.abstractmethod
    def goal_callback(self, goal_request: Any) -> GoalResponse:
        """
        Callback to decide whether to accept or reject a new goal request.
        Should validate the goal parameters and the current state of the system.

        Args:
            goal_request: The goal request message (specific type depends on the action).

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT.
        """
        pass

    # Provide a default implementation which can be overridden if needed
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """
        Callback executed when a goal has been accepted.
        Typically, this initiates the asynchronous execution of the goal and
        updates the parent node's internal state.

        Args:
            goal_handle: The handle for the accepted goal.
        """
        # self.logger.debug(
        #     f"[{self.__class__.__name__}] Goal accepted: {goal_handle.goal_id.uuid.hex}. Starting execution..."
        # )
        # NOTE: Concrete handlers MUST call parent_node.set_internal_state here
        # before calling goal_handle.execute() if using the state machine pattern.
        # Example (should be in concrete handler override or called from it):
        # self.parent_node.set_internal_state(InternalState.EXECUTING_ACTION, goal_handle)
        goal_handle.execute()  # Start the execute_callback

    @abc.abstractmethod
    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        Callback to handle requests to cancel an active goal.

        Args:
            goal_handle: The handle for the goal being canceled.

        Returns:
            CancelResponse.ACCEPT or CancelResponse.REJECT.
        """
        pass

    @abc.abstractmethod
    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Any:
        """
        Asynchronous callback that performs the actual work for the accepted goal.
        This method contains the core logic for the action (e.g., takeoff sequence,
        navigation steps). It should handle feedback publishing, check for
        cancellation, interact with the parent node (e.g., set_target_setpoint, hold_position),
        update the parent node's internal state upon completion/error/cancel,
        and ultimately set the goal status (Succeeded, Aborted, Canceled).

        Args:
            goal_handle: The handle for the goal being executed.

        Returns:
            An instance of the action's Result message (e.g., Takeoff.Result()).
        """
        pass

    @abc.abstractmethod
    async def execute_feedback_callback(self, feedback, goal_handle):
        pass
