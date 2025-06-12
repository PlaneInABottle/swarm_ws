from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch the Swarm Console Interface node in a separate console.
    This console allows users to input parameters for drone swarm control.
    """
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Launch the Swarm Console Interface
    swarm_console_node = Node(
        package='simple_controller',
        executable='swarm_console_node',
        name='swarm_console',
        output='screen',
        emulate_tty=True,  # Ensure console formatting works correctly
    )
    
    # Add the actions to the launch description
    ld.add_action(swarm_console_node)
    
    return ld
