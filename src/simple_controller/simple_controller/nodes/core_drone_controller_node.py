import rclpy
import traceback
import time
import math
import numpy as np
from enum import Enum, auto
from threading import Lock
from typing import Dict, Optional, Type, Any, cast  # Added cast for type checking

# Import specific rclpy types for hints
from rclpy.client import Client
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.action.server import (
    ActionServer,
    ServerGoalHandle,
)  # Import ServerGoalHandle for type hinting
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import lifecycle_msgs.srv
import lifecycle_msgs.msg

# Import PX4 messages
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleRatesSetpoint,  # Example if needed for other handlers
)

# Import Action types from interfaces package
# Replace 'swarm_interfaces' with your actual interface package name
from swarm_interfaces.action import Takeoff

from simple_controller.utils.types import InternalState
from ..utils.avoidance import calculate_simple_avoidance_velocity

# Use try-except for imports if handlers are optional or might not exist initially
try:
    from ..handlers.core_action_handler_base import ActionHandlerBase
    from ..handlers.takeoff_action_handler import TakeoffActionHandler
    from ..handlers.navigate_to_point_action_handler import NavigateToPointActionHandler
    # Import other handlers similarly...
    # from ..handlers.land_action_handler import LandActionHandler
    # from ..handlers.execute_dynamic_motion_handler import ExecuteDynamicMotionHandler
except ImportError as e:
    # Handle the case where handlers might not be found if structure changes
    # This allows the node to potentially still load, but handlers won't work.
    print(
        f"Error importing handlers: {e}. Ensure handlers exist in the correct directory."
    )

    # Define dummy classes if needed to prevent NameErrors later, or raise error
    class ActionHandlerBase:
        pass  # Dummy base

    class TakeoffActionHandler(ActionHandlerBase):
        pass  # Dummy handler

    class NavigateToPointActionHandler(ActionHandlerBase):
        pass  # Dummy handler


# --- Constants ---
DEFAULT_HEARTBEAT_PERIOD = 0.02  # seconds (10 Hz) for OffboardControlMode
SETPOINT_PUBLISH_PERIOD = 0.02  # seconds (20 Hz) for TrajectorySetpoint - MUST BE > 2Hz
CONNECTION_TIMEOUT_SEC = 5.0  # Timeout for waiting for PX4 connection (status)
AVOIDANCE_UPDATE_PERIOD = 0.1


class CoreDroneController(LifecycleNode):
    """
    Core Drone Controller ROS 2 Lifecycle Node.
    Manages direct PX4 communication (Offboard Mode, Setpoints, Commands)
    and hosts Action Servers via dedicated Action Handler classes.
    Uses an internal state machine and continuous setpoint publishing for robustness.
    """

    def __init__(self, **kwargs: Any):
        """Initializes the CoreDroneController node."""
        super().__init__("simple_controller", **kwargs)
        self.get_logger().info("Creating Simple Controller...")

        # --- Parameters (Declare defaults, get in on_configure) ---
        self.declare_parameter("target_system_id", 2)
        self.declare_parameter("px4_ns_prefix", "px4_1")
        self.declare_parameter("offboard_heartbeat_period", DEFAULT_HEARTBEAT_PERIOD)
        self.declare_parameter("setpoint_publish_period", SETPOINT_PUBLISH_PERIOD)

        self.declare_parameter("repulsive_gain", 0.8)
        self.declare_parameter("obstacle_influence_radius", 5.0)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("avoidance_update_period", AVOIDANCE_UPDATE_PERIOD)

        # --- Lifecycle State ---
        self._is_active: bool = False

        # --- Internal State ---
        self._target_system_id: int = 2
        self._px4_ns_prefix: str = "px4_1"
        self._heartbeat_period: float = DEFAULT_HEARTBEAT_PERIOD

        self._is_armed: bool = False
        self._is_offboard: bool = False

        self._desired_position_ned: list[float] = [0.0, 0.0, 0.0]  # x, y, z
        self._desired_velocity_ned: list[float] = [0.0, 0.0, 0.0]  # vx, vy, vz
        self._desired_yaw_rate: float = 0.0  # rad/s
        self._current_local_position: Optional[VehicleLocalPosition] = None
        self._current_global_position: Optional[VehicleGlobalPosition] = None

        # TODO - populate this list
        self._other_drones_local_ned_list: list[np.ndarray] = []

        self._avoidance_repulsion_strength: float = 0.8
        self._avoidance_safety_radius: float = 5.0
        self._avoidance_max_overall_speed: float = 2.0

        # --- Locks ---

        self._setpoint_lock = Lock()  # For thread-safe access to desired setpoints
        self._local_position_lock = Lock()
        self._vehicle_status_lock = Lock()
        self._global_position_lock = Lock()
        self._other_drones_lock = Lock()

        # Internal State Machine
        self._internal_state: InternalState = InternalState.INIT
        self._active_goal_handle: Optional[ServerGoalHandle] = None

        # --- Service Clients ---
        self._vehicle_command_client: Optional[Client] = None

        # --- Action Handlers ---
        self._handlers: dict[str, ActionHandlerBase] = {}

        # --- Callback Groups ---
        self._action_handler_cbgroup = MutuallyExclusiveCallbackGroup()
        self._publisher_cbgroup = ReentrantCallbackGroup()
        self._subscriber_cbgroup = MutuallyExclusiveCallbackGroup()

        # --- Publishers, Subscribers, Timers (Initialized to None) ---

        # If timers left in the default callback_group it blocks the call to this node
        self._offboard_heartbeat_timer: Optional[Timer] = None
        self._setpoint_publishing_timer: Optional[Timer] = None
        self._avoidance_update_timer: Optional[Timer] = None

        self._offboard_control_mode_publisher: Optional[Publisher] = None
        self._trajectory_setpoint_publisher: Optional[Publisher] = None

        self._vehicle_status_subscriber: Optional[Subscription] = None
        self._vehicle_local_position_subscriber: Optional[Subscription] = None
        # self._offboard_heartbeat_timer2 = self.create_timer(
        #   self._heartbeat_period, self._test_callbackv2, callback_group=self._cbgroup
        # )

        # Use try-except during instantiation if a handler class might be missing
        try:
            self.get_logger().info("Attempting to instantiate TakeoffActionHandler...")
            self._handlers["takeoff"] = TakeoffActionHandler(self, self.get_logger())
            self._handlers["navigate_to_point"] = NavigateToPointActionHandler(
                self, self.get_logger()
            )

            self.get_logger().info("Handlers instantiated.")
        except NameError as e:
            self.get_logger().error(
                f"Failed to instantiate handlers, class likely missing: {e}"
            )
            # Node will continue but actions won't work

        self.get_logger().info("Core Drone Controller node created.")

    # === State Management ===
    def set_internal_state(
        self, new_state: InternalState, goal_handle: Optional[ServerGoalHandle] = None
    ) -> None:
        """
        Not At The Moment - Thread-safe method to change the internal operational state.
        Automatically commands hold when transitioning to idle states.
        """
        old_state = self._internal_state
        if old_state != new_state:
            old_state_name = old_state.name
            self.get_logger().info(f"What is the old_state_name: {old_state_name}")
            self._internal_state = new_state

    def get_internal_state(self) -> InternalState:
        return self._internal_state

    # === Lifecycle Implementation ===

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configures resources: parameters, pubs/subs, handlers, timers."""
        self.get_logger().info("Configuring Core Drone Controller...")
        try:
            super().on_configure(state)
            self._target_system_id = int(self.get_parameter("target_system_id").value)
            self._px4_ns_prefix = str(self.get_parameter("px4_ns_prefix").value)
            self._heartbeat_period = float(
                self.get_parameter("offboard_heartbeat_period").value
            )
            self._setpoint_period = float(
                self.get_parameter("setpoint_publish_period").value
            )
            self._avoidance_repulsion_strength = self.get_parameter(
                "repulsive_gain"
            ).value
            self._avoidance_safety_radius = self.get_parameter(
                "obstacle_influence_radius"
            ).value
            self._avoidance_max_overall_speed = self.get_parameter("max_speed").value

            self._avoidance_update_period = float(
                self.get_parameter("avoidance_update_period").value
            )

            # Configure Command Service
            self._vehicle_command_client = self.create_client(
                VehicleCommandSrv, f"/{self._px4_ns_prefix}/fmu/vehicle_command"
            )

            # Wait for the service to be found on the ros network
            while not self._vehicle_command_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"Service /{self._px4_ns_prefix}/fmu/vehicle_command not available, waiting again..."
                )
            self.get_logger().info(
                f"Service /{self._px4_ns_prefix}/fmu/vehicle_command is available."
            )

            # Define QoS Profiles
            qos_pub_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            qos_sub_status = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            qos_subscriber_volatile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            # Configure publishers
            fmu_in_base = f"/{self._px4_ns_prefix}/fmu/in"
            fmu_out_base = f"/{self._px4_ns_prefix}/fmu/out"

            self._offboard_control_mode_publisher = self.create_lifecycle_publisher(
                OffboardControlMode,
                f"{fmu_in_base}/offboard_control_mode",
                qos_pub_reliable,
                callback_group=self._publisher_cbgroup,
            )

            self._trajectory_setpoint_publisher = self.create_lifecycle_publisher(
                TrajectorySetpoint,
                f"{fmu_in_base}/trajectory_setpoint",
                qos_pub_reliable,
                callback_group=self._publisher_cbgroup,
            )

            # Configure subscribers
            self._vehicle_local_position_subscriber = self.create_subscription(
                VehicleLocalPosition,
                f"{fmu_out_base}/vehicle_local_position",
                self._vehicle_local_position_callback,
                qos_subscriber_volatile,
                callback_group=self._subscriber_cbgroup,
            )

            self._vehicle_status_subscriber = self.create_subscription(
                VehicleStatus,
                f"{fmu_out_base}/vehicle_status_v1",
                self._vehicle_status_callback,
                qos_subscriber_volatile,
                callback_group=self._subscriber_cbgroup,
            )

            self._vehicle_global_position_subscriber = self.create_subscription(
                VehicleGlobalPosition,
                f"{fmu_out_base}/vehicle_global_position",
                self._vehicle_global_position_callback,
                qos_subscriber_volatile,
                callback_group=self._subscriber_cbgroup,
            )

            # Configure Timers
            self._offboard_heartbeat_timer = self.create_timer(
                self._heartbeat_period,
                self._publish_offboard_control_mode_heartbeat,
                self._publisher_cbgroup,
            )
            self._offboard_heartbeat_timer.cancel()
            #
            self._setpoint_publishing_timer = self.create_timer(
                self._setpoint_period,
                self._publish_trajectory_setpoint,
                self._publisher_cbgroup,
            )
            self._setpoint_publishing_timer.cancel()

            self._avoidance_update_timer = self.create_timer(
                self._avoidance_update_period,
                self._update_avoidance_logic_callback,
                self._publisher_cbgroup,
            )
            self._avoidance_update_timer.cancel()  # Start it in on_activate as you do

            # Start Action Handlers
            if not self._handlers:
                self.get_logger().warn("No action handlers were instantiated.")
            for name, handler in self._handlers.items():
                # Check if handler is a dummy due to import error
                if (
                    isinstance(handler, ActionHandlerBase)
                    and handler.__class__ != ActionHandlerBase
                ):
                    self.get_logger().info(f"Starting handler: {name}")
                    if not handler.start(
                        callback_group=self._action_handler_cbgroup,
                    ):  # Pass group here
                        self.get_logger().error(f"Failed to start handler: {name}")
                        return TransitionCallbackReturn.FAILURE
                    else:
                        self.get_logger().info(
                            f"on_configure: Handler '{name}' started successfully."
                        )
                else:
                    self.get_logger().warn(
                        f"Skipping start for handler '{name}', possibly due to import error or base class instance."
                    )

            self.get_logger().info("Simple Controller Configured Successfully")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during configuration: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            super().on_activate(state)
            self._is_active = True
            if None in [
                self._offboard_heartbeat_timer,
                self._setpoint_publishing_timer,
            ]:
                self.get_logger().error(
                    "Cannot activate: Not all components were created during configure."
                )
                return TransitionCallbackReturn.FAILURE

            # Use cast to satisfy type checker, we checked for None above
            cast(Timer, self._offboard_heartbeat_timer).reset()
            cast(Timer, self._setpoint_publishing_timer).reset()
            cast(Timer, self._avoidance_update_timer).reset()

            self.get_logger().info("Simple Controller Activated Successfully")

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during activation: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self._is_active = False
            self.get_logger().info("Simple Controller Deactivated Successfully")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during deactivation: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self._is_active = False
            self.get_logger().info("Simple Controller Cleaned Up Successfully")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during cleanup: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self._is_active = False
            self.get_logger().info("Simple Controller Shutdown Successfully")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during shutdown: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self._is_active = False
            self.get_logger().info("Simple Controller Finished On Error Successfully")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during on error: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    # === Get Functions About States ===
    def is_active(self) -> bool:
        return self._is_active

    async def _test_callbackv2(self) -> bool:
        self.get_logger().info("Test CallbackV2")
        return False

    async def _test_callback(self) -> bool:
        self.get_logger().info(f"Test Callback")
        if self._is_active:
            self.get_logger().info("EXECUTED Test Callback")
            self.get_logger().info("EXECUTED V@@@ Callback")
            await self.send_command_request(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
            )

            return self._is_active
        return False

    async def send_command_request(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ):
        """
        Sends a VehicleCommand request to the PX4 FMU.
        """
        try:
            self.get_logger().info(
                f"Sent command {command} with params: [{param1:.2f}, {param2:.2f}, ...]"
            )
            if not self._vehicle_command_client:
                self.get_logger().error(
                    f"Error while sending command {command}: There isn't any client"
                )

            finalRequest = VehicleCommandSrv.Request()
            req = finalRequest.request
            req.timestamp = 0
            req.param1 = float(param1)
            req.param2 = float(param2)
            req.param3 = float(param3)
            req.param4 = float(param4)
            req.param5 = float(param5)
            req.param6 = float(param6)
            req.param7 = float(param7)
            req.command = command
            req.confirmation = 0
            req.target_system = self._target_system_id
            req.target_component = 1
            req.source_system = 255
            req.source_component = 1
            req.from_external = True

            self.get_logger().info(f"Request :  {repr(req)}")
            result = await self._vehicle_command_client.call_async(finalRequest)
            self.get_logger().info(
                f"Sent command {command} with params: [{param1:.2f}, {param2:.2f}, ...]"
            )

            self.get_logger().info(f"Sent command result: {result.reply}")

            return True
        except Exception as e:
            self.get_logger().error(f"Error while sending command {command}: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")

    def _publish_offboard_control_mode_heartbeat(self):
        """Publishes the OffboardControlMode message periodically."""
        if self._offboard_control_mode_publisher is None:
            self.get_logger().info(
                "Heartbeat - Offboard Timer: Publisher not available, skipping.",
                throttle_duration_sec=5.0,
            )
            return

        is_position_hold_intent = self._desired_velocity_ned == [0.0, 0.0, 0.0]

        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True

        if is_position_hold_intent:
            msg.position = True
            msg.velocity = False

        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        try:
            self.get_logger().info(
                f"Heartbeat-Offboard Pub: Pos={msg.position}, Vel={msg.velocity}, Acc={msg.acceleration}, Att={msg.attitude}, Rate={msg.body_rate}",
                throttle_duration_sec=1.0,
            )
            self._offboard_control_mode_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish offboard control mode: {e}", throttle_duration_sec=5
            )

    def _publish_trajectory_setpoint(self):
        """
        Publishes the current TrajectorySetpoint.
        This method is called by a timer and uses the desired setpoints
        (e.g., _desired_velocity_ned, _desired_yaw_rate) that are
        updated by action handlers or other control logic.
        """
        if self._trajectory_setpoint_publisher is None or not self._is_active:
            # self.get_logger().debug( # Optional: for verbose debugging
            #     "Setpoint Timer: Trajectory publisher not available or node inactive, skipping.",
            #     throttle_duration_sec=5.0
            # )
            return

        msg = TrajectorySetpoint()

        # Lock to ensure thread-safe access to the desired setpoint variables
        with self._setpoint_lock:
            is_position_hold_intent = self._desired_velocity_ned == [0.0, 0.0, 0.0]

            if is_position_hold_intent:
                # --- POSITION HOLD MODE ---
                msg.position = [
                    float(self._desired_position_ned[0]),
                    float(self._desired_position_ned[1]),
                    float(self._desired_position_ned[2]),
                ]

                # For position hold with velocity feedforward enabled in OffboardControlMode,
                # send [0,0,0] velocity and 0 yawspeed.
                msg.velocity = [0.0, 0.0, 0.0]
                msg.yawspeed = 0.0
                current_mode_log_str = "POSITION"
                log_details = (
                    f"Pos=[{msg.position[0]:.2f}, {msg.position[1]:.2f}, {msg.position[2]:.2f}], "
                    f"Yaw={msg.yaw:.2f}, VelFF=[0,0,0]"
                )
            else:
                # --- VELOCITY CONTROL MODE ---
                msg.velocity = [
                    float(self._desired_velocity_ned[0]),
                    float(self._desired_velocity_ned[1]),
                    float(self._desired_velocity_ned[2]),
                ]
                msg.yawspeed = float(self._desired_yaw_rate)
                msg.position = [float("nan"), float("nan"), float("nan")]
                current_mode_log_str = "VELOCITY"
                log_details = (
                    f"Vel=[{msg.velocity[0]:.2f}, {msg.velocity[1]:.2f}, {msg.velocity[2]:.2f}], "
                    f"YawRate={msg.yawspeed:.2f}"
                )

        # Acceleration and jerk are usually NaN unless explicitly controlled
        msg.yaw = float("nan")
        msg.acceleration = [float("nan"), float("nan"), float("nan")]
        msg.jerk = [float("nan"), float("nan"), float("nan")]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        try:
            self.get_logger().info(
                f"Publishing TrajectorySetpoint: Vel=[{msg.velocity[0]:.2f}, {msg.velocity[1]:.2f}, {msg.velocity[2]:.2f}], YawRate={msg.yawspeed:.2f}",
                throttle_duration_sec=1.0,
            )
            self._trajectory_setpoint_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish trajectory setpoint: {e}", throttle_duration_sec=5.0
            )

        try:
            self.get_logger().info(
                f"Publishing TrajectorySetpoint: Vel=[{msg.velocity[0]:.2f}, {msg.velocity[1]:.2f}, {msg.velocity[2]:.2f}], YawRate={msg.yawspeed:.2f}",
                throttle_duration_sec=1.0,
            )
            self._trajectory_setpoint_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish trajectory setpoint: {e}", throttle_duration_sec=5.0
            )

    def set_target_velocity_ned(
        self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0
    ):
        """
        Sets the desired velocity in NED frame (m/s) and yaw rate (rad/s).
        This method is called by action handlers or other control logic.
        """
        try:
            with self._setpoint_lock:  #
                self._desired_velocity_ned = [vx, vy, vz]  #
                self._desired_yaw_rate = yaw_rate  #
                self._desired_position_ned = [float("nan"), float("nan"), float("nan")]
            self.get_logger().info(
                f"Target velocity NED set to: [{vx:.2f}, {vy:.2f}, {vz:.2f}], YawRate: {yaw_rate:.2f}",
                throttle_duration_sec=1.0,
            )
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish trajectory setpoint: {e}", throttle_duration_sec=5.0
            )
            return False

    def set_target_position_ned(self, x: float, y: float, z: float):
        try:
            with self._setpoint_lock:  #
                self._desired_position_ned = [x, y, z]  #
                self._desired_velocity_ned = [0.0, 0.0, 0.0]
            self.get_logger().info(
                f"Target Position NED set to: [{x:.2f}, {y:.2f}, {z:.2f}]",
                throttle_duration_sec=1.0,
            )
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to set position trajectory setpoint: {e}",
                throttle_duration_sec=5.0,
            )
            return False

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        with self._local_position_lock:
            self._current_local_position = msg

        self.get_logger().info(
            f"Local POSITION: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}",
            throttle_duration_sec=1.0,
        )

    def _vehicle_status_callback(self, msg: VehicleStatus):
        with self._vehicle_status_lock:
            self._vehicle_status = msg
            self._is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
            self._is_offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        self.get_logger().info(
            f"Is Armed: {self._is_armed}",
            throttle_duration_sec=3.0,
        )
        self.get_logger().info(
            f"Is Offboard: {self._is_offboard}",
            throttle_duration_sec=3.0,
        )

    def _vehicle_global_position_callback(self, msg: VehicleGlobalPosition):
        with self._global_position_lock:
            self._current_global_position = msg
        # Optional: Keep the throttled logging for debugging
        self.get_logger().info(
            f"Global Position Update: Lat={msg.lat:.7f}, Lon={msg.lon:.7f}, Alt_AMSL={msg.alt:.2f}m",
            throttle_duration_sec=2.0,
        )

    def get_current_global_position(self):
        with self._global_position_lock:
            return self._current_global_position

    def is_armed(self):
        return self._is_armed

    def is_offboard(self):
        return self._is_offboard

    def get_current_local_position(self):
        with self._local_position_lock:
            return self._current_local_position

    def _update_avoidance_logic_callback(
        self,
    ):
        if not self._is_active:
            return

        with self._local_position_lock:
            my_current_local_position_msg = self._current_local_position

        is_position_hold_intent = self._desired_velocity_ned == [0.0, 0.0, 0.0]

        if is_position_hold_intent:
            return

        my_current_pos_ned = np.array(
            [
                my_current_local_position_msg.x,
                my_current_local_position_msg.y,
                my_current_local_position_msg.z,
            ]
        )

        # This list needs to be populated by your GPS processing and transformation logic
        with self._other_drones_lock:
            current_other_drones_positions_ned = list(self._other_drones_local_ned_list)

        # --- Determine Base Desired Velocity ---
        # This is what your drone WANTS to do (e.g., formation, waypoint following, hover)
        # For now, let's assume a default hover or a commanded velocity if an action is active.
        # If an action handler (like a formation controller) is active, it should be setting
        # self._desired_velocity_ned and self._desired_yaw_rate.
        # The avoidance will then adjust this.

        current_base_desired_velocity_ned = np.array(
            [0.0, 0.0, 0.0]
        )  # Default to hover
        current_base_yaw_rate = 0.0

        # If action handlers directly update _desired_velocity_ned, we can use that as input
        with self._setpoint_lock:  # Access the values set by handlers/other logic
            current_base_desired_velocity_ned = np.array(self._desired_velocity_ned)
            current_base_yaw_rate = self._desired_yaw_rate

        # Call the actual avoidance function from avoidance.py
        final_velocity_ned = calculate_simple_avoidance_velocity(
            current_position_ned=my_current_pos_ned,
            other_drones_ned_positions=current_other_drones_positions_ned,
            base_desired_velocity_ned=current_base_desired_velocity_ned,
            safety_radius=self._avoidance_safety_radius,  # Using loaded parameter
            repulsion_strength=self._avoidance_repulsion_strength,  # Using loaded parameter
            max_overall_speed=self._avoidance_max_overall_speed,  # Using loaded parameter
        )

        # The avoidance algorithm might change the direction, so re-evaluate yaw if needed,
        # or keep the base_yaw_rate if that's preferred for the maneuver.
        # For simplicity, we'll use the current_base_yaw_rate unless you have specific logic.

        # Update the controller's target velocity with the avoidance-adjusted velocity
        self.set_target_velocity_ned(
            final_velocity_ned[0],
            final_velocity_ned[1],
            final_velocity_ned[2],
            current_base_yaw_rate,  # Or a yaw rate calculated based on final_vel_ned
        )

        self.get_logger().info(
            f"Avoidance Adjusted Vel NED: [{final_velocity_ned[0]:.2f}, {final_velocity_ned[1]:.2f}, {final_velocity_ned[2]:.2f}]",
            throttle_duration_sec=1.0,
        )

    async def engage_arm_mode(self):
        """Sends the ARM command."""
        self.get_logger().info("Sending ARM command...")
        # VEHICLE_CMD_COMPONENT_ARM_DISARM: param1 = 1 to arm, 0 to disarm
        return await self.send_command_request(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    async def engage_offboard_mode(self):
        """Sends command to switch to Offboard flight mode."""
        self.get_logger().info("Sending SET_MODE=OFFBOARD command...")

        # VEHICLE_CMD_DO_SET_MODE: param1 = base_mode (1 for custom mode), param2 = custom_mode (6 for Offboard)
        return await self.send_command_request(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def command_hover_at_current_local_position(self) -> bool:
        """
        Commands the drone to hover at its current local NED position.
        This involves setting the desired position to the current local position
        and zeroing out desired velocities, letting the setpoint publishing
        loop take over to command a position hold.

        Returns:
            bool: True if the command to hover was successfully initiated (i.e., position set),
                  False if local position is unavailable or set_target_position_ned fails.
        """
        current_local_pos = (
            self.get_current_local_position()
        )  # Uses self.get_current_local_position()
        if current_local_pos:
            self.get_logger().info(
                f"Commanding HOVER at local NED: x={current_local_pos.x:.2f}, "
                f"y={current_local_pos.y:.2f}, z={current_local_pos.z:.2f}"
            )
            # Call self.set_target_position_ned() which is part of CoreDroneController
            # This method already sets desired velocity to [0,0,0]
            return self.set_target_position_ned(
                current_local_pos.x, current_local_pos.y, current_local_pos.z
            )
        else:
            self.get_logger().warn(
                "Cannot command hover: Current local position not available. "
                "Attempting to set zero velocity as fallback."
            )
            # Fallback to setting zero velocity if local position is unknown
            return self.set_target_velocity_ned(0.0, 0.0, 0.0, 0.0)


# === Main Execution Logic ===
def main(args: Optional[list[str]] = None):
    """Main function to run the node standalone."""
    print("Starting Simple Controller Node...")
    node: Optional[CoreDroneController] = None
    executor: Optional[MultiThreadedExecutor] = None

    try:
        rclpy.init(args=args)
        node = CoreDroneController()
        node.get_logger().info("[Main] Node instance created.")

        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt, shutting down...")
        else:
            print("Keyboard interrupt, shutting down...")
        if node:
            node.get_logger().info("[Main] KeyboardInterrupt caught.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception in main spin: {e}")
        else:
            print(f"Exception before node init: {e}")
        traceback.print_exc()
        if node:
            node.get_logger().info(f"[Main] Exception caught: {e}")


if __name__ == "__main__":
    main()
