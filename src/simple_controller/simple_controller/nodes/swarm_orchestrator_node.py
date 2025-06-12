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
        self.declare_parameter('drone_count', 3)  # Toplam drone sayısı
        self.declare_parameter('target_height', 25.0)  # Kalkış hedef yüksekliği (m)
        self.declare_parameter('config_file_path', '')  # Koordinat yapılandırma dosyası
        self.declare_parameter('takeoff_retry_count', 5)  # Kalkış deneme sayısı
        self.declare_parameter('retry_delay_sec', 2.0)  # Yeniden deneme beklemesi
        self.declare_parameter('controller_wait_sec', 10.0)  # Controller hazır olana kadar bekleme süresi
        
        self.drone_count = self.get_parameter('drone_count').get_parameter_value().integer_value
        self.target_height = self.get_parameter('target_height').get_parameter_value().double_value
        self.config_path = self.get_parameter('config_file_path').get_parameter_value().string_value
        self.takeoff_retry_count = self.get_parameter('takeoff_retry_count').get_parameter_value().integer_value
        self.retry_delay_sec = self.get_parameter('retry_delay_sec').get_parameter_value().double_value
        self.controller_wait_sec = self.get_parameter('controller_wait_sec').get_parameter_value().double_value

        # Hedef koordinatları yükle
        self._drone_targets = self._load_drone_targets()
        
        # self.get_logger().debug(f"Güçlendirilmiş Akıllı Orkestratör {self.drone_count} drone için başlatıldı.")

        self._get_state_clients: List[Node.create_client] = []
        self._change_state_clients: List[Node.create_client] = []
        self._takeoff_clients: List[ActionClient] = []
        self._navigate_clients: List[ActionClient] = []
        self._drone_states: Dict[int, int] = {}
        self._pending_state_checks = 0

        # İşlem takibi için yeni değişkenler
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
        """drone_targets.yaml dosyasından hedef koordinatları yükler"""
        try:
            # Önce parametre olarak belirtilen dosya yolunu dene
            config_file = self.get_parameter('config_file_path').get_parameter_value().string_value
            
            # Eğer parametre boşsa, varsayılan dosya yolunu kullan
            if not config_file:
                pkg_dir = get_package_share_directory('simple_controller')
                config_file = os.path.join(pkg_dir, 'config', 'drone_targets.yaml')
                
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            targets = config.get('swarm_orchestrator', {}).get('ros__parameters', {}).get('drone_targets', [])
            
            # self.get_logger().debug(f"{len(targets)} hedef koordinat yüklendi: {targets}")
            return targets
        except Exception as e:
            self.get_logger().error(f"Hedef koordinatları yüklenirken hata: {e}")
            return []

    def orchestration_loop(self):
        if self._takeoff_sent:
            # self.get_logger().debug("Orkestrasyon tamamlandı. Döngü durduruluyor.", once=True)
            self.orchestration_timer.cancel()
            return
            
        # self.get_logger().debug("Durum kontrol döngüsü ateşlendi...")
        self._pending_state_checks = self.drone_count
        self._drone_states.clear()

        for i in range(1, self.drone_count + 1):
            client = self._get_state_clients[i-1]
            if not client.service_is_ready():
                self.get_logger().warn(f"Controller-{i} için get_state servisi hazır değil.", throttle_duration_sec=5)
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
            self.get_logger().error(f"Controller-{drone_id} durumu alınırken hata: {e}")
        
        self._pending_state_checks -= 1
        if self._pending_state_checks == 0:
            self.evaluate_system_and_act()

    def evaluate_system_and_act(self):
        if len(self._drone_states) != self.drone_count:
            return

        all_active = all(s == State.PRIMARY_STATE_ACTIVE for s in self._drone_states.values())
        if all_active and not self._takeoff_sent:
            # self.get_logger().debug("Tüm dronlar 'active' durumda. Takeoff başlatılıyor!")
            self._takeoff_sent = True
            self.send_all_takeoff_goals() # Artık async değil
            return

        all_configured_or_active = all(s in [State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE] for s in self._drone_states.values())
        if self._configure_sent and not self._activate_sent and all_configured_or_active:
            if self._activation_timer is None:
                # self.get_logger().debug("Tüm dronlar configure edildi. Activate için 10 saniyelik zamanlayıcı kuruluyor.")
                self._activation_timer = self.create_timer(10.0, self.send_activate_commands)
            self._activate_sent = True 
            return

        if not self._configure_sent:
            if any(s == State.PRIMARY_STATE_UNCONFIGURED for s in self._drone_states.values()):
                # self.get_logger().debug("Configure edilmesi gereken dronlar var. CONFIGURE komutları gönderiliyor...")
                for i in range(1, self.drone_count + 1):
                    if self._drone_states.get(i) == State.PRIMARY_STATE_UNCONFIGURED:
                        self.transition_drone(i, Transition.TRANSITION_CONFIGURE)
            self._configure_sent = True

    def send_activate_commands(self):
        # self.get_logger().debug("10 saniye doldu. Hepsine ACTIVATE gönderiliyor...")
        for i in range(1, self.drone_count + 1):
             if self._drone_states.get(i) == State.PRIMARY_STATE_INACTIVE:
                 self.transition_drone(i, Transition.TRANSITION_ACTIVATE)
        
        if self._activation_timer:
            self._activation_timer.cancel()

    def transition_drone(self, drone_id, transition_id):
        client = self._change_state_clients[drone_id - 1]
        if not client.service_is_ready():
            self.get_logger().error(f"Controller-{drone_id} için change_state servisi hazır değil!")
            return
        
        request = ChangeState.Request()
        request.transition.id = transition_id
        client.call_async(request)
    
    def send_all_takeoff_goals(self):
        """Tüm dronlara kalkış hedefi gönderir - paralel şekilde"""
        self.get_logger().debug("Tüm dronlar için eşzamanlı kalkış hedefi gönderiliyor...")
        
        # Tüm dronlar için arm ve takeoff deneme sayaçlarını sıfırla
        for i in range(1, self.drone_count + 1):
            self._arm_attempts[i] = 0
            self._successful_takeoffs[i] = False

        # Her drone için ayrı bir thread'de takeoff işlemini başlat
        for i in range(self.drone_count):
            drone_id = i + 1
            thread = threading.Thread(
                target=self._send_takeoff_with_retry,
                args=(drone_id,)
            )
            self._takeoff_threads[drone_id] = thread
            thread.start()
            self.get_logger().debug(f"Drone {drone_id} için takeoff thread'i başlatıldı")
            
        # 0.5 saniyelik aralıklarla durumu kontrol et
        self._takeoff_check_timer = self.create_timer(0.5, self._check_takeoff_completion)
        
    def _send_takeoff_with_retry(self, drone_id: int):
        """Bir drone için takeoff komutunu retry mekanizmasıyla gönderir"""
        # _takeoff_clients bir listedir, 0-indeksli erişim için drone_id'den 1 çıkarmalıyız
        client_index = drone_id - 1
        if client_index < 0 or client_index >= len(self._takeoff_clients):
            self.get_logger().error(f"Drone {drone_id} için takeoff client'ı bulunamadı! Geçersiz indeks: {client_index}")
            return
        
        client = self._takeoff_clients[client_index]
        
        # Controller node'unun hizmet vermeye hazır olduğundan emin ol
        start_time = time.time()
        while not client.wait_for_server(timeout_sec=1.0):
            elapsed = time.time() - start_time
            if elapsed > self.controller_wait_sec:
                self.get_logger().error(f"Drone {drone_id} için takeoff action server hazır değil! Zaman aşımı.")
                return
            self.get_logger().debug(f"Drone {drone_id} için takeoff action server bekleniyor... ({elapsed:.1f}s)")
            time.sleep(0.5)
            
        # Mevcut deneme sayısını kontrol et
        with self._client_ready_lock:
            retry_count = self._arm_attempts.get(drone_id, 0)
            
        success = False
        # Başarılı olana kadar ya da maksimum denemeye ulaşana kadar tekrarla
        while not success and retry_count < self.takeoff_retry_count:
            try:
                # Goal mesajı oluştur
                goal_msg = Takeoff.Goal()
                goal_msg.target_height = self.target_height
                goal_msg.takeoff_velocity = 0.3
                
                # Arm deneme sayısını arttır
                with self._client_ready_lock:
                    self._arm_attempts[drone_id] = retry_count + 1
                
                # Goal'u asenkron gönder
                self.get_logger().debug(f"Drone {drone_id} için kalkış hedefi gönderiliyor... (Deneme: {retry_count+1}/{self.takeoff_retry_count})")
                goal_future = client.send_goal_async(goal_msg)
                
                # Future'a callback ekle
                goal_future.add_done_callback(
                    functools.partial(self.takeoff_goal_done_callback, drone_id=drone_id)
                )
                
                # Bu başarılı bir deneme - çıkabiliriz
                success = True
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id} için takeoff gönderilirken hata: {str(e)}")
                retry_count += 1
                if retry_count < self.takeoff_retry_count:
                    self.get_logger().warn(f"Drone {drone_id} için takeoff yeniden denenecek {retry_count}/{self.takeoff_retry_count}")
                    time.sleep(self.retry_delay_sec * (retry_count + 1))  # Her denemede daha uzun bekle
                else:
                    self.get_logger().error(f"Drone {drone_id} için maximum takeoff deneme sayısına ulaşıldı!")
                        
    def _check_takeoff_completion(self):
        """Tüm dronların kalkışının tamamlanıp tamamlanmadığını kontrol eder"""
        # Her dronenun durum bilgisini günlüğe yazarak süreci takip edelim
        takeoff_status = {i: self._successful_takeoffs.get(i, False) for i in range(1, self.drone_count + 1)}
        # self.get_logger().debug(f"Mevcut kalkış durumu: {takeoff_status}")
        
        # Tüm dronların kalkış yapıp yapmadığını kontrol et
        all_complete = all(takeoff_status.values())
        
        if all_complete and not self._navigation_sent:
            # self.get_logger().debug(f"TÜM DRONLAR BAŞARIYLA KALKIŞ YAPTI: {takeoff_status}")
            # self.get_logger().debug("Navigasyon hedefleri 2 saniye içinde gönderilecek...")
            
            # Timer'ın iki kez oluşturulmamasını sağla
            if hasattr(self, '_navigation_timer') and self._navigation_timer is not None:
                self._navigation_timer.cancel()
                
            # Yeni bir timer oluştur ve bunu sakla
            self._navigation_timer = self.create_timer(2.0, self._send_navigation_with_timer_cancel)
            
            # Kontrol döngüsünü durdur
            self._takeoff_check_timer.cancel()
            return
        elif not all_complete:
            # Başarısız olanları kontrol et ve gerekirse yeniden dene
            for drone_id, success in takeoff_status.items():
                if not success and drone_id not in self._takeoff_threads:
                    self.get_logger().warn(f"Drone {drone_id} hala kalkış tamamlamadı. Yeniden deneniyor...")
                    # Kalkış yapamayan drone için yeni bir thread başlat
                    thread = threading.Thread(
                        target=self._send_takeoff_with_retry,
                        args=(drone_id,)
                    )
                    self._takeoff_threads[drone_id] = thread
                    thread.start()

    def takeoff_goal_done_callback(self, future, *, drone_id: int):
        """Takeoff hedefi gönderildikten sonra geri çağrı"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Drone {drone_id} için Takeoff hedefi REDDEDİLDİ!")
            return
            
        # self.get_logger().debug(f"Drone {drone_id} için Takeoff hedefi KABUL EDİLDİ.")
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(functools.partial(self.takeoff_result_callback, drone_id=drone_id))

    def takeoff_result_callback(self, future, *, drone_id: int):
        """Takeoff tamamlandıktan sonra geri çağrı"""
        try:
            result = future.result().result
            
            if result.success:
                # Sonucu günlüğe yaz, final_height var mı kontrol et
                try:
                    height_str = f", Yükseklik: {result.final_height:.2f}m"
                except AttributeError:
                    height_str = " (Yükseklik bilgisi mevcut değil)"
                    
                # self.get_logger().debug(f"Drone {drone_id} takeoff BAŞARILI{height_str}")
                
                # Takeoff başarılı olduğunda başarı durumunu kaydet
                self._successful_takeoffs[drone_id] = True
                
                # Tüm dronların durumunu kontrol et ve debug için görüntüle
                all_success = all(self._successful_takeoffs.get(i, False) for i in range(1, self.drone_count + 1))
                # self.get_logger().debug(f"Takeoff durumu: {self._successful_takeoffs}, Tüm dronlar hazır mı: {all_success}")
                
                if all_success and not self._navigation_sent:
                    # self.get_logger().debug("TÜM DRONLAR BAŞARIYLA KALKIŞ YAPTI. Navigasyon başlatılıyor...")
                    # 2 saniye bekleyelim ve sonra navigasyon hedeflerini gönderelim
                    self._navigation_timer = self.create_timer(2.0, self._send_navigation_with_timer_cancel)
            else:
                error_msg = f"Drone {drone_id} takeoff BAŞARISIZ."
                if hasattr(result, 'message'):
                    error_msg += f" Mesaj: {result.message}"
                
                self.get_logger().error(error_msg)
                
                # Başarısız takeoff'u tekrar denemek için
                if not self._navigation_sent:
                    self.get_logger().warn(f"Drone {drone_id} için takeoff BAŞARISIZ - TEKRAR DENEME {self._arm_attempts.get(drone_id, 0)}/{self.takeoff_retry_count}")
                    # Biraz daha uzun bekle, belki drone hazır olacak
                    time.sleep(self.retry_delay_sec * 2)
                    # Thread'i temizle ve yeni başlat
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
                self.get_logger().warn(f"Drone {drone_id} için takeoff hata sonrası yeniden denenecek...")
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
        """Navigasyon gönderiminden önce timer'ı iptal eden yardımcı fonksiyon"""
        # Timer'ı iptal et (oneshot simülasyonu için)
        if hasattr(self, '_navigation_timer'):
            self._navigation_timer.cancel()
        
        # Navigasyon hedeflerini göndermek yerine formasyon yöneticisini başlat
        # self.get_logger().debug("Formasyon yöneticisi başlatılıyor...")
        
        # Formation manager'ı başlat
        self.launch_formation_manager()
        
        # Eski navigasyon kodu yorum satırına alındı
        # self.send_all_navigation_goals()
        
    def send_all_navigation_goals(self):
        """Tüm dronlara paralel şekilde navigasyon hedefi gönderir"""
        self._navigation_sent = True
        # self.get_logger().debug("Tüm dronlar için eşzamanlı navigasyon hedefi gönderiliyor...")
        
        # Her drone için ayrı thread başlat
        for i in range(self.drone_count):
            drone_id = i + 1
            thread = threading.Thread(
                target=self._send_navigation_goal,
                args=(drone_id,)
            )
            self._navigate_threads[drone_id] = thread
            thread.start()
            # self.get_logger().debug(f"Drone {drone_id} için navigasyon thread'i başlatıldı")
    
    def _send_navigation_goal(self, drone_id: int):
        """Belirli bir drone için navigasyon hedefini gönderir"""
        client = self._navigate_clients[drone_id-1]
        
        # Action sunucusunun hazır olmasını bekle
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f"Drone {drone_id} için navigasyon aksiyon sunucusu hazır değil!")
            return
            
        try:
            # Hedef koordinatları kontrol et (1-tabanlı indeks değil, indeks-1 kullanılmalı)
            index = drone_id - 1
            if index >= len(self._drone_targets):
                self.get_logger().error(f"Drone {drone_id} için hedef koordinatlar bulunamadı!")
                return
            
            # Yükseklik sınır parametresini oku (varsayılan: 10 metre)
            max_alt_limit = self.get_parameter("max_alt_limit").value if \
                self.has_parameter("max_alt_limit") else 10.0
                
            # Hedef goal'u oluştur
            goal = NavigateToPoint.Goal()
            drone_target = self._drone_targets[index]
            
            # YAML dosyasındaki anahtar adlarına göre değerleri ayarla
            goal.target_latitude_deg = drone_target['lat']
            goal.target_longitude_deg = drone_target['lon']
            
            # Yükseklik sınır kontrolü - takeoff'tan sonra en fazla max_alt_limit kadar yükselebilir
            target_alt = drone_target['alt_amsl']
            takeoff_height = self.get_parameter("target_height").value
            base_alt = 300.0  # Varsayılan taban yüksekliği
            
            # Hedef yükseklik kontrolü, taban + takeoff + max_alt_limit'i geçemez
            limited_alt = min(target_alt, base_alt + takeoff_height + max_alt_limit)
            goal.target_altitude_amsl = limited_alt
            
            goal.desired_travel_speed_mps = drone_target.get('speed_mps', 1.5)
            goal.acceptance_radius_m = drone_target.get('radius_m', 1.0)
            
            # self.get_logger().debug(f"Drone {drone_id} için navigasyon hedefi gönderiliyor: "
            #                        f"lat: {goal.target_latitude_deg:.7f}, lon: {goal.target_longitude_deg:.7f}, "
            #                        f"alt: {goal.target_altitude_amsl:.1f} m, hız: {goal.desired_travel_speed_mps} m/s")
            
            # Goal'u asenkron olarak gönder
            goal_future = client.send_goal_async(goal)
            goal_future.add_done_callback(functools.partial(self.navigation_goal_done_callback, drone_id=drone_id))
            
        except Exception as e:
            self.get_logger().error(f"Drone {drone_id} için navigasyon hedefi gönderilirken hata: {str(e)}")
            return

    def navigation_goal_done_callback(self, future, *, drone_id: int):
        """Navigasyon hedefi gönderildikten sonra geri çağrı"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Drone {drone_id} için Navigasyon hedefi REDDEDİLDİ!")
            return
            
        # self.get_logger().debug(f"Drone {drone_id} için Navigasyon hedefi KABUL EDİLDİ.")
        
        # Sonuç için geri çağrı ekle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self.navigation_result_callback, drone_id=drone_id)
        )
    
    def navigation_result_callback(self, future, drone_id: int):
        """Navigasyon sonuç callback'i"""
        try:
            result = future.result()
            
            if result.success:
                # self.get_logger().debug(f'Drone {drone_id} için navigasyon BAŞARILI: {result.message}')
                pass
            else:
                self.get_logger().error(f'Drone {drone_id} için navigasyon BAŞARISIZ: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Drone {drone_id} için navigasyon sonucu alınırken hata: {str(e)}')
    
    def launch_formation_manager(self):
        """Formasyon yöneticisini başlatır"""
        # self.get_logger().debug("===== TÜM DRONELAR BAŞARIYLA KALKIŞ YAPTILAR =====")
        # self.get_logger().debug("===== FORMASYON YÖNETİCİSİ BAŞLATILIYOR =====")
        
        # Formasyon yöneticisi parametreleri
        formation_type = "v"  # Varsayılan olarak V formasyonu
        drone_spacing = 5.0   # Dronlar arası mesafe (metre)
        enable_formation = True  # Formasyon kontrolünü aktifleştir
        
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
            
            # self.get_logger().debug(f"Formasyon yöneticisi başlatılıyor: {' '.join(cmd)}")
            
            # Komutu yeni bir terminalde çalıştır (gnome-terminal ya da xterm)
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