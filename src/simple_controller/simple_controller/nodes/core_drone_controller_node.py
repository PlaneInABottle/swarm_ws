import rclpy
import traceback
import time
import math
from enum import Enum, auto
from threading import Lock
from typing import Dict, Optional, Type, Any, cast  # Added cast for type checking

# Import specific rclpy types for hints
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
    VehicleRatesSetpoint,  # Example if needed for other handlers
)

# Import Action types from interfaces package
# Replace 'swarm_interfaces' with your actual interface package name
from swarm_interfaces.action import Takeoff

from simple_controller.utils.types import InternalState

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
    print(
        f"Error importing handlers: {e}. Ensure handlers exist in the correct directory."
    )

    # Define dummy classes if needed to prevent NameErrors later, or raise error
    class ActionHandlerBase:
        pass  # Dummy base

    class TakeoffActionHandler(ActionHandlerBase):
        pass  # Dummy handler


# --- Constants ---
DEFAULT_HEARTBEAT_PERIOD = 1  # seconds (10 Hz) for OffboardControlMode
SETPOINT_PUBLISH_PERIOD = 0.05  # seconds (20 Hz) for TrajectorySetpoint - MUST BE > 2Hz
CONNECTION_TIMEOUT_SEC = 5.0  # Timeout for waiting for PX4 connection (status)


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
        self.declare_parameter("px4_ns_prefix", "px4_1")  # Example, should match drone
        self.declare_parameter("offboard_heartbeat_period", DEFAULT_HEARTBEAT_PERIOD)

        # --- Lifecycle State ---
        self._is_active: bool = False

        # --- Internal State ---
        self._target_system_id: int = 2
        self._px4_ns_prefix: str = "px4_1"
        self._heartbeat_period: float = DEFAULT_HEARTBEAT_PERIOD

        # Internal State Machine
        self._internal_state: InternalState = InternalState.INIT
        self._active_goal_handle: Optional[ServerGoalHandle] = (
            None  # Store the currently active goal
        )

        # --- Service Clients ---
        # Change type later
        self._vehicle_command_client: Any = None

        # --- Action Handlers ---
        self._handlers: dict[str, ActionHandlerBase] = {}

        # --- Callback Groups ---
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self._action_handler_cbgroup = MutuallyExclusiveCallbackGroup()
        self._action_handler_feedback_cbgroup = ReentrantCallbackGroup()
        self._publisher_cbgroup = MutuallyExclusiveCallbackGroup()

        # Timers
        # If left in the default callback_group it blocks the call to this node
        # --- Publishers, Subscribers, Timers (Initialized to None) ---
        self._offboard_control_mode_pub: Optional[Publisher] = None
        self._offboard_heartbeat_timer: Optional[Timer] = None
        # self._offboard_heartbeat_timer2 = self.create_timer(
        #   self._heartbeat_period, self._test_callbackv2, callback_group=self._cbgroup
        # )

        # Use try-except during instantiation if a handler class might be missing
        try:
            self.get_logger().debug("Attempting to instantiate TakeoffActionHandler...")
            self._handlers["takeoff"] = TakeoffActionHandler(self, self.get_logger())
            self.get_logger().debug("TakeoffActionHandler instantiated.")
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
            self._target_system_id = int(self.get_parameter("target_system_id").value)
            self._px4_ns_prefix = str(self.get_parameter("px4_ns_prefix").value)
            self._heartbeat_period = float(
                self.get_parameter("offboard_heartbeat_period").value
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
            qos_sub_pose = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            # Configure publishers
            fmu_in_base = f"/{self._px4_ns_prefix}/fmu/in"
            self._offboard_control_mode_pub = self.create_lifecycle_publisher(
                OffboardControlMode,
                f"{fmu_in_base}/offboard_control_mode",
                qos_pub_reliable,
            )

            # Configure Heartbeat - Offboard Timer (paused)
            self._offboard_heartbeat_timer = self.create_timer(
                self._heartbeat_period,
                self._publish_offboard_control_mode_heartbeat,
                self._publisher_cbgroup,
            )
            self._offboard_heartbeat_timer.cancel()
            #
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
                        feedback_callback_group=self._action_handler_feedback_cbgroup,
                    ):  # Pass group here
                        self.get_logger().error(f"Failed to start handler: {name}")
                        return (
                            TransitionCallbackReturn.FAILURE
                        )  # Indicate configuration failed
                    else:
                        self.get_logger().debug(
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
            self._is_active = True
            if None in [self._offboard_heartbeat_timer]:
                self.get_logger().error(
                    "Cannot activate: Not all components were created during configure."
                )
                return TransitionCallbackReturn.FAILURE

            # Use cast to satisfy type checker, we checked for None above
            cast(Timer, self._offboard_heartbeat_timer).reset()

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
            # Asynchronously call the service

            return True
        except Exception as e:
            self.get_logger().error(f"Error while sending command {command}: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")

    def _publish_offboard_control_mode_heartbeat(self):
        """Publishes the OffboardControlMode message periodically."""
        if self._offboard_control_mode_pub is None:
            self.get_logger().info(
                "Heartbeat - Offboard Timer: Publisher not available, skipping.",
                throttle_duration_sec=5.0,
            )
            return

        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False  # Set true if handlers use velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        try:
            self.get_logger().debug(
                f"Heartbeat-Offboard Pub: Pos={msg.position}, Vel={msg.velocity}, Acc={msg.acceleration}, Att={msg.attitude}, Rate={msg.body_rate}",
                throttle_duration_sec=1.0,
            )
            self._offboard_control_mode_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish offboard control mode: {e}", throttle_duration_sec=5
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
        # PX4 requires continuous setpoints BEFORE this command is accepted.
        self.get_logger().info("Sending SET_MODE=OFFBOARD command...")

        # VEHICLE_CMD_DO_SET_MODE: param1 = base_mode (1 for custom mode), param2 = custom_mode (6 for Offboard)
        return await self.send_command_request(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )


# === Main Execution Logic ===
def main(args: Optional[list[str]] = None):
    """Main function to run the node standalone."""
    print("Starting Simple Controller Node...")
    node: Optional[CoreDroneController] = None
    executor: Optional[MultiThreadedExecutor] = None

    try:
        rclpy.init(args=args)
        node = CoreDroneController()
        node.get_logger().debug("[Main] Node instance created.")

        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt, shutting down...")
        else:
            print("Keyboard interrupt, shutting down...")
        if node:
            node.get_logger().debug("[Main] KeyboardInterrupt caught.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception in main spin: {e}")
        else:
            print(f"Exception before node init: {e}")
        traceback.print_exc()
        if node:
            node.get_logger().debug(f"[Main] Exception caught: {e}")


if __name__ == "__main__":
    main()
