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
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv

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
    print(f"Error importing handlers: {e}. Ensure handlers exist in the correct directory.")
    # Define dummy classes if needed to prevent NameErrors later, or raise error
    class ActionHandlerBase: pass # Dummy base
    class TakeoffActionHandler(ActionHandlerBase): pass # Dummy handler


# --- Constants ---
DEFAULT_HEARTBEAT_PERIOD = 1 # seconds (10 Hz) for OffboardControlMode
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
        super().__init__('simple_controller', **kwargs)
        self.get_logger().info("Creating Simple Controller...")

        # --- Parameters (Declare defaults, get in on_configure) ---
        self.declare_parameter('target_system_id', 2)
        self.declare_parameter('px4_ns_prefix', 'px4_1') # Example, should match drone
        self.declare_parameter('offboard_heartbeat_period', DEFAULT_HEARTBEAT_PERIOD)

        # --- Lifecycle State ---
        self._is_active : bool = False

        # --- Internal State ---
        self._target_system_id: int = 2
        self._px4_ns_prefix: str = 'px4_1'
        self._heartbeat_period: float = DEFAULT_HEARTBEAT_PERIOD

        # Internal State Machine
        self._internal_state: InternalState = InternalState.INIT
        self._active_goal_handle: Optional[ServerGoalHandle] = None # Store the currently active goal

        # --- Action Handlers ---
        self._handlers: dict[str, ActionHandlerBase] = {}

        # --- Action Handlers ---
        # Change type later
        self._vehicle_command_client: Any = None
        self._cbgroup = MutuallyExclusiveCallbackGroup()

        # Timers
        self._offboard_heartbeat_timer = self.create_timer(self._heartbeat_period, self._test_callback)

        try:
            self.get_logger().debug("Attempting to instantiate TakeoffActionHandler...")
            self._handlers['takeoff'] = TakeoffActionHandler(self, self.get_logger())
            self.get_logger().debug("TakeoffActionHandler instantiated.")
        except NameError as e:
             self.get_logger().error(f"Failed to instantiate handlers, class likely missing: {e}")
             # Node will continue but actions won't work

        self.get_logger().info("Core Drone Controller node created.")
        self.get_logger().debug("Node __init__ complete.")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:

            self._target_system_id = int(self.get_parameter('target_system_id').value)
            self._px4_ns_prefix = str(self.get_parameter('px4_ns_prefix').value)
            self._heartbeat_period = float(self.get_parameter('offboard_heartbeat_period').value)

            self._vehicle_command_client_name = f"/{self._px4_ns_prefix}/fmu/vehicle_command"
            self._vehicle_command_client = self.create_client(VehicleCommandSrv, '/px4_1/fmu/vehicle_command')
            # Wait for the service to be found on the ros network
            while not self._vehicle_command_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service /{self._px4_ns_prefix}/fmu/vehicle_command not available, waiting again...')

            self.get_logger().info("Simple Controller Configured Successfully")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Exception during configuration: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR 

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self._is_active = True

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

    async def _test_callback(self) -> bool:
        self.get_logger().info(f"Test Callback")
        if self._is_active:
            self.get_logger().info(f"EXECUTED Test Callback")
            await self.send_command_request(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

            return self._is_active
        return False

    async def send_command_request(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0, param4: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0):
        """
        Sends a VehicleCommand request to the PX4 FMU.
        """
        try:
            self.get_logger().info(f"Sent command {command} with params: [{param1:.2f}, {param2:.2f}, ...]")
            if not self._vehicle_command_client :
                self.get_logger().error(f"Error while sending command {command}: There isn't any client")

            req = VehicleCommandSrv.Request()
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

            future = await self._vehicle_command_client.call_async(req)
            self.get_logger().info(f"Sent command {command} with params: [{param1:.2f}, {param2:.2f}, ...]")
            
            self.get_logger().info(f"Sent command result: {future.result()}")

            # Asynchronously call the service
            
            return True
        except Exception as e:
            self.get_logger().error(f"Error while sending command {command}: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")



# === Main Execution Logic ===
def main(args: Optional[list[str]] = None):
     """ Main function to run the node standalone. """
     print('Starting Simple Controller Node...')
     node: Optional[CoreDroneController] = None
     executor: Optional[MultiThreadedExecutor] = None

     try:
         rclpy.init(args=args)
         node = CoreDroneController()
         node.get_logger().debug("[Main] Node instance created.")

         executor = MultiThreadedExecutor(num_threads = 2)
         executor.add_node(node)
         executor.spin()

     except KeyboardInterrupt:
         if node: node.get_logger().info('Keyboard interrupt, shutting down...')
         else: print('Keyboard interrupt, shutting down...')
         if node: node.get_logger().debug("[Main] KeyboardInterrupt caught.")
     except Exception as e:
          if node: node.get_logger().error(f"Exception in main spin: {e}")
          else: print(f"Exception before node init: {e}")
          traceback.print_exc()
          if node: node.get_logger().debug(f"[Main] Exception caught: {e}")

if __name__ == '__main__':
     main()
