import rclpy
import time
import math
import functools
from rclpy.action.server import GoalResponse, CancelResponse, ServerGoalHandle
from rclpy.lifecycle import LifecycleState, LifecycleNode
from rclpy.duration import Duration
from typing import Type, Any, Optional

from rclpy.task import Future

# Import the base class
from .core_action_handler_base import ActionHandlerBase

# Import the specific action type
# Make sure this matches your package and action definition
try:
    from swarm_interfaces.action import NavigateToPoint
except ImportError:
    print(
        "CRITICAL: Could not import NavigateToPoint action from swarm_interfaces.action."
    )
    print("Ensure the action is defined and the package is built and sourced.")

    # Define a dummy action if not found, to allow loading, but it won't work.
    class NavigateToPoint:
        Goal = type("Goal", (), {})
        Result = type("Result", (), {})
        Feedback = type("Feedback", (), {})


# Import the InternalState enum from the node definition (if used for state checking)
try:
    from simple_controller.utils.types import InternalState
except ImportError:
    from enum import Enum, auto

    class InternalState(Enum):  # Dummy
        IDLE_HOVERING = auto()
        EXECUTING_ACTION = auto()
        UNKNOWN = auto()


# Constants for NavigateToPointActionHandler
DEFAULT_TRAVEL_SPEED_MPS = 1.5  # m/s
DEFAULT_ACCEPTANCE_RADIUS_M = (
    0.75  # meters - How close to target is considered 'reached'
)
EARTH_RADIUS_METERS = 6378137.0  # For distance calculations

# Define a minimum time between feedback publications to avoid spamming
MIN_FEEDBACK_INTERVAL_SEC = 0.5  # Publish feedback at most every 0.5 seconds


class NavigateToPointActionHandler(ActionHandlerBase):
    """
    Action Handler for the NavigateToPoint action.
    Manages navigating the drone to a specified GPS coordinate.
    """

    def __init__(self, parent_node: LifecycleNode, logger: Any):
        """
        Initialize the NavigateToPointActionHandler.

        Args:
            parent_node: The CoreDroneController node.
            logger: The logger instance from the parent node.
        """
        super().__init__(
            parent_node=parent_node,
            logger=logger,
            action_name="navigate_to_point",  # Action server name
            action_type=NavigateToPoint,  # Action type definition
        )
        # Store target information during execution
        self._target_lat_deg: Optional[float] = None
        self._target_lon_deg: Optional[float] = None
        self._target_alt_amsl: Optional[float] = None
        self._desired_speed_mps: Optional[float] = None
        self._acceptance_radius_m: Optional[float] = None

    def goal_callback(self, goal_request: NavigateToPoint.Goal) -> GoalResponse:
        """
        Accept or reject a NavigateToPoint goal request.
        """
        self.logger.info(
            f"[{self.__class__.__name__}] Received NavigateToPoint goal: "
            f"Lat={goal_request.target_latitude_deg:.6f}, Lon={goal_request.target_longitude_deg:.6f}, "
            f"Alt={goal_request.target_altitude_amsl:.2f}m, Speed={goal_request.desired_travel_speed_mps:.2f}m/s, "
            f"Radius={goal_request.acceptance_radius_m:.2f}m"
        )

        if not self.is_started or self._action_server is None:
            self.logger.error(
                "Rejecting goal: NavigateToPoint action server not started."
            )
            return GoalResponse.REJECT

        # Basic validation of goal parameters
        if goal_request.acceptance_radius_m <= 0:
            self.logger.error("Rejecting goal: Acceptance radius must be positive.")
            return GoalResponse.REJECT

        # Check if parent node is in a state that allows navigation (e.g., armed and hovering)
        # This depends on your CoreDroneController's state management
        # Example:
        # if self.parent_node.get_internal_state() not in [InternalState.IDLE_HOVERING, InternalState.EXECUTING_ACTION]:
        #     self.logger.error(f"Rejecting goal: Drone not in a state for navigation (current: {self.parent_node.get_internal_state()}).")
        #     return GoalResponse.REJECT

        if not self.parent_node.is_armed():
            self.logger.error("Rejecting goal: Drone is not armed.")
            return GoalResponse.REJECT

        if not self.parent_node.is_offboard():
            self.logger.error("Rejecting goal: Drone is not in offboard mode.")
            # Optionally, you could try to switch to offboard mode here if appropriate for your design
            return GoalResponse.REJECT

        self.logger.info("Accepting NavigateToPoint goal.")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Start the asynchronous execution."""
        self.logger.info(
            f"[{self.__class__.__name__}] Goal accepted: {goal_handle.goal_id}. Starting execution..."
        )
        # self.parent_node.set_internal_state(InternalState.EXECUTING_ACTION) # Example
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.logger.info(
            f"[{self.__class__.__name__}] Received cancel request for goal {goal_handle.goal_id}."
        )
        # Call the parent_node's method
        self.parent_node.command_hover_at_current_local_position()
        self.logger.info(
            "Cancellation: Commanded drone to hover at current local position via parent."
        )
        return CancelResponse.ACCEPT

    async def execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> NavigateToPoint.Result:
        """
        Executes the NavigateToPoint action.
        """
        result = NavigateToPoint.Result()
        feedback_msg = NavigateToPoint.Feedback()  # Renamed to avoid conflict
        request: NavigateToPoint.Goal = goal_handle.request

        self.execute_future = Future()

        # Store goal parameters for use in the feedback loop
        self._target_lat_deg = request.target_latitude_deg
        self._target_lon_deg = request.target_longitude_deg
        self._target_alt_amsl = request.target_altitude_amsl
        self._desired_speed_mps = (
            request.desired_travel_speed_mps
            if request.desired_travel_speed_mps > 0
            else DEFAULT_TRAVEL_SPEED_MPS
        )
        self._acceptance_radius_m = (
            request.acceptance_radius_m
            if request.acceptance_radius_m > 0
            else DEFAULT_ACCEPTANCE_RADIUS_M
        )

        self.logger.info(
            f"Executing NavigateToPoint: Target Lat={self._target_lat_deg:.6f}, Lon={self._target_lon_deg:.6f}, "
            f"Alt={self._target_alt_amsl:.2f}m, Speed={self._desired_speed_mps:.2f}m/s, "
            f"Radius={self._acceptance_radius_m:.2f}m"
        )

        # --- Pre-flight checks (similar to Takeoff, if any specific needed before starting movement) ---
        current_global_pos = self.parent_node.get_current_global_position()
        if current_global_pos is None:
            result.success = False
            result.message = "Failed to get current global position at action start."
            self.logger.error(result.message)
            goal_handle.abort()
            # self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Example
            return result

        # Ensure drone is armed and in offboard (already checked in goal_callback, but good for safety)
        if not self.parent_node.is_armed() or not self.parent_node.is_offboard():
            result.success = False
            result.message = (
                "Drone disarmed or not in offboard mode during execution start."
            )
            self.logger.error(result.message)
            # Call the parent_node's method
            self.parent_node.command_hover_at_current_local_position()
            goal_handle.abort()
            return result

        # --- Start Feedback Timer and Await Completion ---
        partial_callback = functools.partial(
            self.execute_feedback_callback, feedback_msg, goal_handle
        )
        # self.execute_timer_period is inherited from ActionHandlerBase
        self.execute_feedback_timer = self.parent_node.create_timer(
            self.execute_timer_period,
            partial_callback,
            callback_group=self._callback_group,
        )

        action_outcome = await self.execute_future

        # --- Clean up ---
        if self.execute_feedback_timer:
            self.execute_feedback_timer.cancel()
        self.execute_feedback_timer = None
        self.execute_future = None

        # Reset stored target info
        self._target_lat_deg = None
        self._target_lon_deg = None
        self._target_alt_amsl = None
        self._desired_speed_mps = None
        self._acceptance_radius_m = None
        self._last_feedback_time = None

        # --- Process Result ---
        final_pos = self.parent_node.get_current_global_position()
        if final_pos:
            result.final_latitude_deg = final_pos.lat
            result.final_longitude_deg = final_pos.lon
            result.final_altitude_amsl = final_pos.alt
        else:  # Should ideally not happen if we reached here
            result.final_latitude_deg = 0.0
            result.final_longitude_deg = 0.0
            result.final_altitude_amsl = 0.0

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "NavigateToPoint canceled by request."
            goal_handle.canceled()
            self.logger.info(result.message)
        elif action_outcome and action_outcome.get("success", False):
            result.success = True
            result.message = action_outcome.get(
                "message", "NavigateToPoint successful."
            )
            goal_handle.succeed()
            self.logger.info(result.message)
        else:
            result.success = False
            result.message = action_outcome.get("message", "NavigateToPoint failed.")
            goal_handle.abort()
            self.logger.error(result.message)

        # self.parent_node.set_internal_state(InternalState.IDLE_HOVERING) # Example
        return result

    async def execute_feedback_callback(
        self, feedback_msg: NavigateToPoint.Feedback, goal_handle: ServerGoalHandle
    ):
        """
        Periodically called to update drone's velocity towards the target,
        publish feedback, and check for goal completion or cancellation.
        """
        if (
            not self.is_started
            or self._target_lat_deg is None
            or self._target_lon_deg is None
            or self._target_alt_amsl is None
            or self._desired_speed_mps is None
            or self._acceptance_radius_m is None
            # Ensure future is active and not already done
            or self.execute_future is None
            or self.execute_future.done()
        ):
            self.logger.warn(
                "NavigateToPoint not properly initialized, future missing, or future done. Skipping feedback."
            )
            return

        # --- 1. Handle Cancellation ---
        if goal_handle.is_cancel_requested:
            if self.execute_future and not self.execute_future.done():
                self.execute_future.set_result(
                    {
                        "success": False,
                        "message": "Navigation canceled during execution.",
                    }
                )
            # Hover command is usually handled in cancel_callback, but can be reinforced here if needed
            # self.parent_node.set_target_velocity_ned(0.0, 0.0, 0.0, 0.0)
            return

        # --- 2. Get Current State ---
        current_global_pos = self.parent_node.get_current_global_position()
        if current_global_pos is None:
            self.logger.warn(
                "Cannot get current global position in feedback loop. Commanding hover."
            )
            # Call the parent_node's method
            self.parent_node.command_hover_at_current_local_position()
            # ... (optional: abort if GPS lost for too long) ...
            return

        # --- 3. Check Drone Status (Arm, Offboard) ---
        if not self.parent_node.is_armed() or not self.parent_node.is_offboard():
            self.logger.error("Drone disarmed or lost offboard mode during navigation!")
            if not self.execute_future.done():
                self.execute_future.set_result(
                    {
                        "success": False,
                        "message": "Drone disarmed or lost offboard during navigation.",
                    }
                )
            # Call the parent_node's method
            self.parent_node.command_hover_at_current_local_position()
            return

        # --- 4. Calculate Distance and Vector to Target ---
        current_lat_rad = math.radians(current_global_pos.lat)
        current_lon_rad = math.radians(current_global_pos.lon)
        current_alt_amsl = current_global_pos.alt

        target_lat_rad = math.radians(self._target_lat_deg)
        target_lon_rad = math.radians(self._target_lon_deg)

        # Calculate displacements in meters (NED frame components)
        delta_lat_rad = target_lat_rad - current_lat_rad
        delta_lon_rad = target_lon_rad - current_lon_rad

        delta_north_m = EARTH_RADIUS_METERS * delta_lat_rad
        # Approximation for delta_east_m, good for small steps
        delta_east_m = EARTH_RADIUS_METERS * delta_lon_rad * math.cos(current_lat_rad)
        # delta_down_m: Positive Down. Target_AMSL - Current_AMSL gives "Up" distance. Negate for "Down".
        delta_down_m = -(self._target_alt_amsl - current_alt_amsl)

        distance_3d_to_target = math.sqrt(
            delta_north_m**2 + delta_east_m**2 + delta_down_m**2
        )

        # --- 5. Publish Feedback ---
        feedback_msg.current_latitude_deg = current_global_pos.lat
        feedback_msg.current_longitude_deg = current_global_pos.lon
        feedback_msg.current_altitude_amsl = current_alt_amsl
        feedback_msg.distance_to_target_m = distance_3d_to_target
        goal_handle.publish_feedback(feedback_msg)
        self.logger.debug(f"Nav Feedback: DistToTarget={distance_3d_to_target:.2f}m")

        # --- 6. Check for Completion ---
        if distance_3d_to_target <= self._acceptance_radius_m:
            self.logger.info(
                f"Target GPS point reached within acceptance radius ({self._acceptance_radius_m:.2f}m). "
                f"Distance: {distance_3d_to_target:.2f}m."
            )
            self.parent_node.command_hover_at_current_local_position()
            if not self.execute_future.done():
                self.execute_future.set_result(
                    {
                        "success": True,
                        "message": "Navigation successful, target reached.",
                    }
                )
            return  # Goal achieved

        # ******************************************************************* #
        # --- 7. ADDED/CRITICAL: Command Drone Movement (If not completed) --- #
        # ******************************************************************* #
        if (
            distance_3d_to_target > 1e-3
        ):  # Avoid division by zero if very close but not caught by acceptance
            # Normalize the direction vector
            # (delta_north_m, delta_east_m, delta_down_m already represent the vector to target)
            dir_n = delta_north_m / distance_3d_to_target
            dir_e = delta_east_m / distance_3d_to_target
            dir_d = delta_down_m / distance_3d_to_target

            # Calculate velocity components based on desired speed
            vel_n = self._desired_speed_mps * dir_n
            vel_e = self._desired_speed_mps * dir_e
            vel_d = (
                self._desired_speed_mps * dir_d
            )  # vz is positive downwards in NED for PX4

            # Yaw control:
            # For simplicity, maintaining current heading (yaw_rate = 0.0).
            # A more advanced implementation could calculate yaw_rate to face the direction of travel:
            #   desired_yaw_angle = math.atan2(vel_e, vel_n)
            #   (current_yaw would be needed from local_position.heading)
            #   yaw_error = shortest_angular_distance(current_yaw, desired_yaw_angle)
            #   yaw_rate_cmd = Kp_yaw * yaw_error
            #   (ensure yaw_rate_cmd is capped)
            yaw_rate_cmd = 0.0

            self.parent_node.set_target_velocity_ned(vel_n, vel_e, vel_d, yaw_rate_cmd)
            self.logger.debug(
                f"Commanding Velocity NED: N={vel_n:.2f}, E={vel_e:.2f}, D={vel_d:.2f}, YawRate={yaw_rate_cmd:.2f}"
            )
        else:
            # This case should ideally be caught by the acceptance radius check,
            # but as a final safeguard if it's extremely close.
            self.logger.info("Effectively at target, commanding hover.")
            self.parent_node.command_hover_at_current_local_position()
