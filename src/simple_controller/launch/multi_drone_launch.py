import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launches Gazebo with multiple drones and their corresponding controller nodes
    by calling a helper script.
    """
    num_drones = 3
    package_share_dir = get_package_share_directory('simple_controller')
    script_path = os.path.join(package_share_dir, 'scripts', 'px4_starter.sh')
    
    launch_actions = []

    for i in range(1, num_drones + 1):
        instance_id = i
        position_x = 10.0 + (i - 1) * 2.0
        position_y = 10.0
        
        # Her bir drone için PX4 SITL sürecini yeni bir terminalde başlat
        px4_process = ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 
                script_path, 
                str(instance_id), 
                str(position_x), 
                str(position_y)
            ],
            shell=False # Script doğrudan çalıştırıldığı için shell=False daha güvenli
        )
        
        # Her bir drone için ROS 2 kontrolcüsünü başlat
        controller_node = Node(
            package='simple_controller',
            executable='core_drone_controller_node',
            name=f'controller_{instance_id}',
            parameters=[
                {'target_system_id': instance_id},
                {'px4_ns_prefix': f'px4_{instance_id}'}
            ],
            output='screen'
        )
        
        launch_actions.append(px4_process)
        launch_actions.append(controller_node)

    return LaunchDescription(launch_actions)