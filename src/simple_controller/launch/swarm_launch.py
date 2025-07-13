import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

# =============================================================================
# === HELPER FUNCTION: Yeni bir terminalde komut çalıştırmak için
# =============================================================================
def run_in_new_terminal(command, title=""):
    try:
        find_executable = FindExecutable(name='gnome-terminal')
        command_prefix = [find_executable, '--', 'bash', '-c']
    except Exception:
        find_executable = FindExecutable(name='xterm')
        command_prefix = [find_executable, '-T', title, '-e', 'bash', '-c']
    
    full_command = command_prefix + [f"echo '===== {title} ====='; {command}; exec bash"]
    return ExecuteProcess(cmd=full_command, shell=False, output='screen')

# =============================================================================
# === LAUNCH DESCRIPTION
# =============================================================================
def generate_launch_description():
    ld = LaunchDescription([SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', 'fastdds.xml')])
    
    DRONE_COUNT = 3
    px4_executable_path = os.path.join(os.path.expanduser('~'), 'PX4-Autopilot/build/px4_sitl_default/bin/px4')
    px4_base_cmd = f"HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500"

    ld = LaunchDescription()

    # =================================================================================
    # === ADIM 1: PX4 SİMÜLASYONLARINI İSTENİLEN ZAMANLAMAYLA BAŞLATMA
    # =================================================================================
    
    # PX4-1: Hemen başlasın (t=0s)
    ld.add_action(TimerAction(period=0.0, actions=[
        LogInfo(msg="PX4-1 başlatılıyor..."),
        run_in_new_terminal(f"{px4_base_cmd} PX4_GZ_MODEL_POSE='0,0' {px4_executable_path} -i 1", title="PX4-1")
    ]))
    
    # PX4-2 ve PX4-3: birbirlerinden daha fazla zaman aralıklarıyla başlasınlar
    ld.add_action(TimerAction(period=15.0, actions=[
        LogInfo(msg="PX4-2 başlatılıyor... (15 saniye sonra)"),
        run_in_new_terminal(f"PX4_GZ_STANDALONE=1 {px4_base_cmd} PX4_GZ_MODEL_POSE='3,3' {px4_executable_path} -i 2", title="PX4-2")
    ]))
    
    ld.add_action(TimerAction(period=25.0, actions=[
        LogInfo(msg="PX4-3 başlatılıyor... (25 saniye sonra)"),
        run_in_new_terminal(f"PX4_GZ_STANDALONE=1 {px4_base_cmd} PX4_GZ_MODEL_POSE='6,6' {px4_executable_path} -i 3", title="PX4-3")
    ]))

    # =================================================================================
    # === ADIM 2: CONTROLLER DÜĞÜMLERİNİ SIRAYLA BAŞLATMA
    # =================================================================================

    controller_start_time = 40.0
    controller_interval = 5.0 

    for i in range(1, DRONE_COUNT + 1):
        current_controller_delay = controller_start_time + (i - 1) * controller_interval
        controller_cmd = (
            f"ros2 run simple_controller core_drone_controller_node "
            f"--ros-args -r __node:=controller_{i} "
            f"-p px4_ns_prefix:=px4_{i} "
            f"-p target_system_id:={i + 1}"
        )
        ld.add_action(TimerAction(
            period=current_controller_delay,
            actions=[
                LogInfo(msg=f"Controller-{i} başlatılıyor... ({current_controller_delay:.1f} saniye sonra)"),
                run_in_new_terminal(controller_cmd, title=f"Controller-{i}")
            ]
        ))

    # =================================================================================
    # === ADIM 3: AKILLI ORKESTRATÖRÜ BAŞLATMA
    # =================================================================================
    # Son controller başladıktan sonra orkestratörün başlaması için beklenecek süre
    # Son controller t = 20s + (3-1)*3s = 26s'de başlar.
    # Orchestrator 10 saniye sonra t=36s'de başlatalım.
    orchestrator_start_time = controller_start_time + (DRONE_COUNT * controller_interval) + 15.0

    ld.add_action(TimerAction(
        period=orchestrator_start_time,
        actions=[
            LogInfo(msg=f"Orchestrator Initialize ({orchestrator_start_time:.1f} saniye sonra)"),
            Node(
                package='simple_controller',
                executable='swarm_orchestrator_node',
                name='swarm_orchestrator',
                output='screen', # Ana terminalde calısması icin
                parameters=[
                    {'drone_count': DRONE_COUNT},
                    {'target_height': 25.0},
                    {'config_file_path': os.path.join(get_package_share_directory('simple_controller'), 'config', 'drone_targets.yaml')}
                ]
            )
        ]
    ))
    
    return ld
