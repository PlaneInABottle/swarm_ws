import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.action.client import ClientGoalHandle
from typing import List, Dict, Optional, Any
import functools
import yaml
import os
import threading
import time
import subprocess
import signal
from ament_index_python.packages import get_package_share_directory

from swarm_interfaces.action import Takeoff, NavigateToPoint
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
from ament_index_python.packages import get_package_share_directory

class SwarmOrchestrator(Node):
    def __init__(self):
        super().__init__('swarm_orchestrator')
        self.declare_parameter('drone_count', 3)  
        self.declare_parameter('target_height', 25.0) 
        self.declare_parameter('config_file_path', '')  # Koordinat yapılandırma dosyası
        self.declare_parameter('takeoff_retry_count', 5) 
        self.declare_parameter('retry_delay_sec', 2.0)  # Yeniden deneme beklemesi
        self.declare_parameter('controller_wait_sec', 10.0)  # Controller hazır olana kadar bekleme
        
        self.drone_count = self.get_parameter('drone_count').get_parameter_value().integer_value
        self.target_height = self.get_parameter('target_height').get_parameter_value().double_value
        self.config_path = self.get_parameter('config_file_path').get_parameter_value().string_value
        self.takeoff_retry_count = self.get_parameter('takeoff_retry_count').get_parameter_value().integer_value
        self.retry_delay_sec = self.get_parameter('retry_delay_sec').get_parameter_value().double_value
        self.controller_wait_sec = self.get_parameter('controller_wait_sec').get_parameter_value().double_value
        
        self._drone_targets = self._load_drone_targets()
        

        self._get_state_clients: List[Node.create_client] = []
        self._change_state_clients: List[Node.create_client] = []
        self._takeoff_clients: List[ActionClient] = []
        self._navigate_clients: List[ActionClient] = []
        self._drone_states: Dict[int, int] = {}
        self._pending_state_checks = 0
        self._configure_sent = False
        self._activate_sent = False
        self._activation_timer: Optional[rclpy.timer.Timer] = None
        self._takeoff_sent = False
        self._navigation_sent = False
        self._successful_takeoffs: Dict[int, bool] = {}
        self._arm_attempts: Dict[int, int] = {}
        self._client_ready_lock = threading.Lock()
        self._takeoff_threads: Dict[int, threading.Thread] = {}
        self._navigate_threads: Dict[int, threading.Thread] = {}

        for i in range(1, self.drone_count + 1):
            controller_name = f'controller_{i}'
            self._get_state_clients.append(self.create_client(GetState, f'/{controller_name}/get_state'))
            self._change_state_clients.append(self.create_client(ChangeState, f'/{controller_name}/change_state'))
            self._takeoff_clients.append(ActionClient(self, Takeoff, f'/{controller_name}/takeoff'))
            self._navigate_clients.append(ActionClient(self, NavigateToPoint, f'/{controller_name}/navigate_to_point'))
            self._successful_takeoffs[i] = False
        
        self.orchestration_timer = self.create_timer(2.0, self.orchestration_loop)
        
    def _load_drone_targets(self) -> List[Dict[str, Any]]:
        try:
            config_file = self.get_parameter('config_file_path').get_parameter_value().string_value
            if not config_file:
                pkg_dir = get_package_share_directory('simple_controller')
                config_file = os.path.join(pkg_dir, 'config', 'drone_targets.yaml')
                
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            targets = config.get('swarm_orchestrator', {}).get('ros__parameters', {}).get('drone_targets', [])
            return targets
        except Exception as e:
            self.get_logger().error(f"Hedef coordinates hata: {e}")
            return []

    def orchestration_loop(self):
        if self._takeoff_sent:
            self.orchestration_timer.cancel()
            return
        self._pending_state_checks = self.drone_count
        self._drone_states.clear()

        for i in range(1, self.drone_count + 1):
            client = self._get_state_clients[i-1]
            if not client.service_is_ready():
                self._pending_state_checks -= 1
                self.get_logger().warn(f"Controller-{i} için get_state servisi hazdegil.", throttle_duration_sec=5)
                self._pending_state_checks -= 1
                continue
            
            request = GetState.Request()
            future = client.call_async(request)
            future.add_done_callback(functools.partial(self.process_state_update, drone_id=i))

    def process_state_update(self, future, *, drone_id: int):
        try:
            result = future.result()
            if result:
                self._drone_states[drone_id] = result.current_state.id
        except Exception as e:
            self.get_logger().error(f"Controller-{drone_id} durumu alinirken hata: {e}")
        
        self._pending_state_checks -= 1
        if self._pending_state_checks == 0:
            self.evaluate_system_and_act()

    def evaluate_system_and_act(self):
        if len(self._drone_states) != self.drone_count:
            return

        all_active = all(s == State.PRIMARY_STATE_ACTIVE for s in self._drone_states.values())
        if all_active and not self._takeoff_sent:
            self._takeoff_sent = True
            self.send_all_takeoff_goals() 
            return

        all_configured_or_active = all(s in [State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE] for s in self._drone_states.values())
        if self._configure_sent and not self._activate_sent and all_configured_or_active:
            if self._activation_timer is None:
                self._activation_timer = self.create_timer(10.0, self.send_activate_commands)
            self._activate_sent = True 
            return

        if not self._configure_sent:
            if any(s == State.PRIMARY_STATE_UNCONFIGURED for s in self._drone_states.values()):
                for i in range(1, self.drone_count + 1):
                    if self._drone_states.get(i) == State.PRIMARY_STATE_UNCONFIGURED:
                        self.transition_drone(i, Transition.TRANSITION_CONFIGURE)
            self._configure_sent = True

    def send_activate_commands(self):
        for i in range(1, self.drone_count + 1):
             if self._drone_states.get(i) == State.PRIMARY_STATE_INACTIVE:
                 self.transition_drone(i, Transition.TRANSITION_ACTIVATE)
        
        if self._activation_timer:
            self._activation_timer.cancel()

    def transition_drone(self, drone_id, transition_id):
        client = self._change_state_clients[drone_id - 1]
        if not client.service_is_ready():
            self.get_logger().error(f"Controller-{drone_id} için change_state servisi hazir degil!")
            return
        
        request = ChangeState.Request()
        request.transition.id = transition_id
        client.call_async(request)
    
    def send_all_takeoff_goals(self):
        for i in range(1, self.drone_count + 1):
            self._arm_attempts[i] = 0
            self._successful_takeoffs[i] = False

        for i in range(self.drone_count):
            drone_id = i + 1
            thread = threading.Thread(
                target=self._send_takeoff_with_retry,
                args=(drone_id,)
            )
            self._takeoff_threads[drone_id] = thread
            thread.start()
        self._takeoff_check_timer = self.create_timer(0.5, self._check_takeoff_completion)
        
    def _send_takeoff_with_retry(self, drone_id: int):
        client_index = drone_id - 1
        if client_index < 0 or client_index >= len(self._takeoff_clients):
            self.get_logger().error(f"Drone {drone_id} için takeoff client'i bulunamadi: {client_index}")
            return
        
        client = self._takeoff_clients[client_index]
        start_time = time.time()
        while not client.wait_for_server(timeout_sec=1.0):
            elapsed = time.time() - start_time
            if elapsed > self.controller_wait_sec:
                self.get_logger().error(f"Drone {drone_id} için takeoff action server hazir degil! Zaman asimi.")
                return
            time.sleep(0.5)

        with self._client_ready_lock:
            retry_count = self._arm_attempts.get(drone_id, 0)
            
        success = False
        while not success and retry_count < self.takeoff_retry_count:
            try:
         
                goal_msg = Takeoff.Goal()
                goal_msg.target_height = self.target_height
                goal_msg.takeoff_velocity = 0.3
                
              
                with self._client_ready_lock:
                    self._arm_attempts[drone_id] = retry_count + 1
                
                goal_future = client.send_goal_async(goal_msg)
                
             
                goal_future.add_done_callback(
                    functools.partial(self.takeoff_goal_done_callback, drone_id=drone_id)
                )
                
                success = True
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id} icin takeoff gonderilirken hata: {str(e)}")
                retry_count += 1
                if retry_count < self.takeoff_retry_count:
                    self.get_logger().warn(f"Drone {drone_id} icin takeoff yeniden denenecek {retry_count}/{self.takeoff_retry_count}")
                    time.sleep(self.retry_delay_sec * (retry_count + 1))  # Her denemede daha uzun bekle
                else:
                    self.get_logger().error(f"Drone {drone_id} icin maximum takeoff deneme sayisina ")
                        
    def _check_takeoff_completion(self):
        takeoff_status = {i: self._successful_takeoffs.get(i, False) for i in range(1, self.drone_count + 1)}
        all_complete = all(takeoff_status.values())
        
        if all_complete and not self._navigation_sent:
            if hasattr(self, '_navigation_timer') and self._navigation_timer is not None:
                self._navigation_timer.cancel()
                
            self._navigation_timer = self.create_timer(2.0, self._send_navigation_with_timer_cancel)

            self._takeoff_check_timer.cancel()
            return
        elif not all_complete:
        
            for drone_id, success in takeoff_status.items():
                if not success and drone_id not in self._takeoff_threads:
                    self.get_logger().warn(f"Drone {drone_id} hala kalkis tamamlamadi. Yeniden deneniyor...")
     
                    thread = threading.Thread(
                        target=self._send_takeoff_with_retry,
                        args=(drone_id,)
                    )
                    self._takeoff_threads[drone_id] = thread
                    thread.start()

    def takeoff_goal_done_callback(self, future, *, drone_id: int):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Drone {drone_id} için Takeoff hedefi reddedildi")
            return
            
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(functools.partial(self.takeoff_result_callback, drone_id=drone_id))

    def takeoff_result_callback(self, future, *, drone_id: int):
        try:
            result = future.result().result
            
            if result.success:
               
                try:
                    height_str = f", Yukseklik: {result.final_height:.2f}m"
                except AttributeError:
                    height_str = " (Yukseklik bilgisi mevcut degil)"
                    
                
                self._successful_takeoffs[drone_id] = True
                
            
                all_success = all(self._successful_takeoffs.get(i, False) for i in range(1, self.drone_count + 1))
              
                if all_success and not self._navigation_sent:
           
                    self._navigation_timer = self.create_timer(2.0, self._send_navigation_with_timer_cancel)
            else:
                error_msg = f"Drone {drone_id} takeoff basarisiz"
                if hasattr(result, 'message'):
                    error_msg += f" Mesaj: {result.message}"
                
                self.get_logger().error(error_msg)
                
                if not self._navigation_sent:
                    self.get_logger().warn(f"Drone {drone_id} için takeoff BASRISIZ - TEKRAR DENEME {self._arm_attempts.get(drone_id, 0)}/{self.takeoff_retry_count}")
            
                    time.sleep(self.retry_delay_sec * 2)
               
                    if drone_id in self._takeoff_threads:
                        del self._takeoff_threads[drone_id]
                    thread = threading.Thread(
                        target=self._send_takeoff_with_retry,
                        args=(drone_id,)
                    )
                    self._takeoff_threads[drone_id] = thread
                    thread.start()
        except Exception as e:
            self.get_logger().error(f"Takeoff result callback'de hata: {str(e)}")
            # Hata durumunda da takeoff mekanizmasını tetikle
            if not self._navigation_sent:
                self.get_logger().warn(f"Drone {drone_id} icin takeoff hata sonrasi yeniden denenecek...")
                # Thread'i temizle ve yeni bir thread başlat
                if drone_id in self._takeoff_threads:
                    del self._takeoff_threads[drone_id]
                thread = threading.Thread(
                    target=self._send_takeoff_with_retry,
                    args=(drone_id,)
                )
                self._takeoff_threads[drone_id] = thread
                thread.start()


    def _send_navigation_with_timer_cancel(self):
        # Timer'ı iptal et (oneshot simulasyonu için)
        if hasattr(self, '_navigation_timer'):
            self._navigation_timer.cancel()
        
        self.launch_formation_manager()
        
        
    def send_all_navigation_goals(self):
        self._navigation_sent = True

        for i in range(self.drone_count):
            drone_id = i + 1
            thread = threading.Thread(
                target=self._send_navigation_goal,
                args=(drone_id,)
            )
            self._navigate_threads[drone_id] = thread
            thread.start()

    def _send_navigation_goal(self, drone_id: int):
        """Belirli bir drone icin navigasyon hedefini gonderir"""
        client = self._navigate_clients[drone_id-1]
        
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f"Drone {drone_id} icin navigasyon aksiyon sunucusu hazir degil!")
            return
            
        try:
           
            index = drone_id - 1
            if index >= len(self._drone_targets):
                self.get_logger().error(f"Drone {drone_id} için hedef koordinatlar bulunamadi!")
                return
            
     
            max_alt_limit = self.get_parameter("max_alt_limit").value if \
                self.has_parameter("max_alt_limit") else 10.0
    
            goal = NavigateToPoint.Goal()
            drone_target = self._drone_targets[index]
            
   
            goal.target_latitude_deg = drone_target['lat']
            goal.target_longitude_deg = drone_target['lon']

            target_alt = drone_target['alt_amsl']
            takeoff_height = self.get_parameter("target_height").value
            base_alt = 300.0  
         
            limited_alt = min(target_alt, base_alt + takeoff_height + max_alt_limit)
            goal.target_altitude_amsl = limited_alt
            
            goal.desired_travel_speed_mps = drone_target.get('speed_mps', 1.5)
            goal.acceptance_radius_m = drone_target.get('radius_m', 1.0)
      
            goal_future = client.send_goal_async(goal)
            goal_future.add_done_callback(functools.partial(self.navigation_goal_done_callback, drone_id=drone_id))
            
        except Exception as e:
            self.get_logger().error(f"Drone {drone_id} icin navigasyon hedefi gonderilirken hata: {str(e)}")
            return

    def navigation_goal_done_callback(self, future, *, drone_id: int):
    
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Drone {drone_id} icin Navigasyon hedefi REDDEDiLDi!")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self.navigation_result_callback, drone_id=drone_id)
        )
    
    def navigation_result_callback(self, future, drone_id: int):
        try:
            result = future.result()
            
            if result.success:
                pass
            else:
                self.get_logger().error(f'Drone {drone_id} için navigasyon BAŞARISIZ: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Drone {drone_id} için navigasyon sonucu alınırken hata: {str(e)}')
    
    def launch_formation_manager(self):

        formation_type = "v"  # default olarak V formasyonu
        drone_spacing = 5.0   # Dronlar arası mesafe (metre)
        enable_formation = True  # Formasyon kontrolunu aktiflestir
        
        try:
            # Formation launch dosyasının yolunu bul
            pkg_dir = get_package_share_directory('simple_controller')
            
            # Formasyon yöneticisini başlat
            cmd = [
                'ros2', 'launch', 'simple_controller', 'formation_launch.py',
                f'formation_type:={formation_type}',
                f'drone_spacing:={drone_spacing}',
                f'enable_formation:={str(enable_formation).lower()}'
            ]

            try:
                term_cmd = ['gnome-terminal', '--', 'bash', '-c', ' '.join(cmd) + '; exec bash']
                self._formation_process = subprocess.Popen(term_cmd)
                # self.get_logger().debug("Formasyon yöneticisi yeni terminalde başlatıldı (gnome-terminal)")
            except Exception:
                try:
                    term_cmd = ['xterm', '-T', 'Formation Manager', '-e', 'bash -c "' + ' '.join(cmd) + '; exec bash"']
                    self._formation_process = subprocess.Popen(term_cmd)
                    # self.get_logger().debug("Formasyon yöneticisi yeni terminalde başlatıldı (xterm)")
                except Exception as e:
                    # Doğrudan başlatmayı dene (terminal olmadan)
                    self._formation_process = subprocess.Popen(cmd)
                    # self.get_logger().debug("Formasyon yöneticisi doğrudan başlatıldı")
            
            # Node sonlandığında formasyon yöneticisini de sonlandır
            # self.get_logger().debug("Formation manager başlatıldı. PID: " + str(self._formation_process.pid))
            
        except Exception as e:
            self.get_logger().error(f"Formation manager başlatılırken hata: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        orchestrator = SwarmOrchestrator()
        executor = MultiThreadedExecutor()
        executor.add_node(orchestrator)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            orchestrator.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
if __name__ == '__main__':
    main()