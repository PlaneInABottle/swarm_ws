#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from swarm_interfaces.action import NavigateToPoint
from swarm_interfaces.msg import SwarmState, DroneIndividualState
from threading import Lock
import threading
import time
import numpy as np
from typing import List, Dict, Any, Optional
from enum import Enum, auto

# Yerel modüller
from ..utils.formation_factory import FormationFactory


class FormationType(Enum):
    """
    Desteklenen formasyon tipleri
    """
    V_FORMATION = auto()
    LINE_FORMATION = auto()
    ARROWHEAD_FORMATION = auto()


class FormationManager(Node):
    """
    Formasyon yöneticisi node'u. 
    - SwarmState topic'ini dinler
    - Seçilen formasyon tipine göre koordinatlar hesaplar
    - Munkres algoritması ile drone'ları hedeflere atar
    - NavigateToPoint action client ile drone'ları yönlendirir
    """

    def __init__(self):
        super().__init__('formation_manager')

        # Parametreler
        self.declare_parameter('formation_type', 'v')  # v, line, arrowhead
        self.declare_parameter('drone_spacing', 5.0)   # metre cinsinden
        self.declare_parameter('enable_formation', False)  # Başlangıçta pasif
        self.declare_parameter('navigation_acceptance_radius', 1.0)  # metre
        self.declare_parameter('navigation_speed', 0.5)  # m/s

        # Parametreleri oku
        self._formation_type_str = self.get_parameter('formation_type').get_parameter_value().string_value
        self._drone_spacing = self.get_parameter('drone_spacing').get_parameter_value().double_value
        self._enable_formation = self.get_parameter('enable_formation').get_parameter_value().bool_value
        self._navigation_acceptance_radius = self.get_parameter('navigation_acceptance_radius').get_parameter_value().double_value
        self._navigation_speed = self.get_parameter('navigation_speed').get_parameter_value().double_value

        # Formation tipini belirle
        if self._formation_type_str.lower() == 'v':
            self._formation_type = FormationType.V_FORMATION
        elif self._formation_type_str.lower() == 'line':
            self._formation_type = FormationType.LINE_FORMATION
        elif self._formation_type_str.lower() == 'arrowhead':
            self._formation_type = FormationType.ARROWHEAD_FORMATION
        else:
            self._formation_type = FormationType.V_FORMATION  # Varsayılan
            self.get_logger().warn(f"Tanınmayan formasyon tipi: '{self._formation_type_str}'. V formasyonu kullanılıyor.")

        # Sürü durum verileri
        self._swarm_state_lock = Lock()
        self._drones = []
        self._swarm_centroid = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self._initial_centroid = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self._initial_centroid_set = False  # İlk merkez kaydedildi mi?
        self._last_formation_update = 0.0
        self._formation_update_rate = 1.0  # CPU tasarrufu için 2.0'dan 1.0'a düşürüldü
        
        # Callback grupları
        self._subscription_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_callback_group = MutuallyExclusiveCallbackGroup()
        self._timer_callback_group = ReentrantCallbackGroup()

        # QoS profili
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # SwarmState topic'ine abone ol
        self._swarm_state_sub = self.create_subscription(
            SwarmState,
            '/swarm_state',
            self._swarm_state_callback,
            qos_profile,
            callback_group=self._subscription_callback_group
        )
        
        # Formasyon hesaplama ve atama işlemi için zamanlayıcı
        self._formation_timer = self.create_timer(
            1.0 / self._formation_update_rate,  # saniye başına güncelleme
            self._formation_update_callback,
            callback_group=self._timer_callback_group
        )

        # Action clientları (her drone için)
        self._action_clients = {}  # {drone_id: NavigateToPointClient}
        self._navigation_futures = {}  # {drone_id: Future}
        self._last_target_positions = {}  # {drone_id: {'lat': lat, 'lon': lon, 'alt': alt}}
        
        # Servisler, parametreler için callback'ler
        self.add_on_set_parameters_callback(self._parameters_callback)
        
        # self.get_logger().debug(f"Formasyon Yöneticisi başlatıldı: "
        #                      f"Formasyon: {self._formation_type_str}, "
        #                      f"Mesafe: {self._drone_spacing}m, "
        #                      f"Aktif: {self._enable_formation}")

    def _parameters_callback(self, params):
        # Parametre değişikliklerini işle
        for param in params:
            if param.name == 'formation_type':
                self._formation_type_str = param.value
                if param.value.lower() == 'v':
                    self._formation_type = FormationType.V_FORMATION
                elif param.value.lower() == 'line':
                    self._formation_type = FormationType.LINE_FORMATION
                elif param.value.lower() == 'arrowhead':
                    self._formation_type = FormationType.ARROWHEAD_FORMATION
                else:
                    self._formation_type = FormationType.V_FORMATION
                    self.get_logger().warn(f"Tanınmayan formasyon tipi: '{param.value}'. V formasyonu kullanılıyor.")
                # self.get_logger().debug(f"Formasyon tipi değiştirildi: {param.value}")
            
            elif param.name == 'drone_spacing':
                self._drone_spacing = param.value
                # self.get_logger().debug(f"Drone mesafesi değiştirildi: {param.value}m")
            
            elif param.name == 'enable_formation':
                self._enable_formation = param.value
                state_str = "Aktif" if param.value else "Pasif"
                # self.get_logger().debug(f"Formasyon durumu değiştirildi: {state_str}")
                
            elif param.name == 'navigation_acceptance_radius':
                self._navigation_acceptance_radius = param.value
            
            elif param.name == 'navigation_speed':
                self._navigation_speed = param.value
                
        return True  # Tüm parametreler kabul edildi

    def _swarm_state_callback(self, msg: SwarmState):
        """SwarmState mesajları için callback"""
        with self._swarm_state_lock:
            # Drone listesini güncelle - yalnızca ihtiyacımız olan bilgileri saklayarak CPU ve bellek kullanımını azaltıyoruz
            self._drones = []
            valid_positions_count = 0
            
            for drone in msg.drones:
                # Yalnızca aktif olarak kullandığımız alanları ekle
                drone_data = {
                    'id': drone.drone_id,
                    'lat': drone.latitude,
                    'lon': drone.longitude,
                    'alt': drone.altitude,
                    'global_position_valid': drone.global_position_valid
                }
                self._drones.append(drone_data)
                
                if drone.global_position_valid:
                    valid_positions_count += 1
            
            # Sürü merkezi bilgisini güncelle (anlık merkez için - GPS koordinatlarını kullan)
            self._swarm_centroid = {
                'lat': msg.centroid_latitude,  # GPS latitude
                'lon': msg.centroid_longitude, # GPS longitude
                'alt': msg.centroid_altitude   # GPS altitude
            }
            
            # İlk sürü merkezi kaydedilmemişse ve geçerli konum verileri varsa kaydet
            if not self._initial_centroid_set and valid_positions_count > 0 and msg.centroid_latitude != 0.0:
                # Kalkış öncesi ilk merkez kaydediliyor (GPS koordinatlarını kullan)
                self._initial_centroid = self._swarm_centroid.copy()
                self._initial_centroid_set = True
                # self.get_logger().debug(f"Initial swarm centroid saved: lat={self._initial_centroid['lat']:.7f}, "
                #                        f"lon={self._initial_centroid['lon']:.7f}, alt={self._initial_centroid['alt']:.2f}m")
            
        # Her drone için action client oluştur (eğer yoksa)
        # Bu işlemi _swarm_state_callback dışına taşıyoruz, çünkü her callback'te client'ları kontrol etmek CPU kullanımını artırır
        # Bunun yerine formasyon güncellemesinde bir kez yapıyoruz

    def _formation_update_callback(self):
        """Formasyon güncelleme zamanlayıcısı callback'i"""
        # CPU kullanımını azaltmak için erken return kontrolleri
        if not self._enable_formation:
            return  # Formasyon kontrolü devre dışı
            
        # Drone verileri ve sürü merkezi var mı kontrol et
        with self._swarm_state_lock:
            if not self._drones or self._swarm_centroid['lat'] == 0.0:
                return  # Veri henüz gelmemiş
            
            # Her drone için action client oluştur (eğer yoksa)
            for drone in self._drones:
                drone_id = drone['id']
                if drone_id not in self._action_clients:
                    # Action client oluştur
                    self._action_clients[drone_id] = ActionClient(
                        self,
                        NavigateToPoint,
                        f'/controller_{drone_id}/navigate_to_point',
                        callback_group=self._action_callback_group
                    )
                
            # Drone verilerini ve sürü merkezini kopyala - shallow copy kullanarak CPU kullanımını azalt
            drones = list(self._drones) # deep copy yerine shallow copy
            swarm_centroid = dict(self._swarm_centroid)
        
        # Eğer ilk merkez kaydedilmediyse, mevcut swarm_centroid kullan
        # Aksi halde sabit ilk merkezi kullan
        formation_center = self._initial_centroid if self._initial_centroid_set else swarm_centroid
        
        # Formasyon noktalarını sabit ilk merkez etrafında hesapla
        target_positions = self._calculate_formation_points(
            formation_center['lat'],
            formation_center['lon'],
            formation_center['alt']  # Yüksekliği güncel tutabiliriz, opsiyonel
        )
        
        if not target_positions or len(target_positions) < len(drones):
            self.get_logger().warn(f"Formasyon pozisyonları hesaplanamadı veya yetersiz")
            return
            
        # Drone'ları hedeflere ata
        assignments = FormationFactory.assign_drones_to_targets(drones, target_positions)
        
        # İlk merkez kaydedildiyse formatı rapor et (debug amaçlı)
        if self._initial_centroid_set and self.get_logger().get_effective_level() <= 10: # DEBUG level
            formation_type_str = "Bilinmiyor"
            if self._formation_type == FormationType.V_FORMATION:
                formation_type_str = "V Formasyonu"
            elif self._formation_type == FormationType.LINE_FORMATION:
                formation_type_str = "Line Formasyonu"
            elif self._formation_type == FormationType.ARROWHEAD_FORMATION:
                formation_type_str = "Arrowhead Formasyonu"
                
            # self.get_logger().debug(
            #     f"Formasyon oluşturuluyor: {formation_type_str}, "
            #     f"Merkez: lat={formation_center['lat']:.6f}, lon={formation_center['lon']:.6f}"
            # )
        
        # Her drone için navigasyon komutu gönder
        for drone_id, target in assignments:
            self._send_navigation_command(drone_id, target)
    
    def _calculate_formation_points(self, center_lat, center_lon, center_alt):
        """Seçilen formasyon tipine göre hedef pozisyonları hesaplar"""
        if self._formation_type == FormationType.V_FORMATION:
            return FormationFactory.calculate_v_formation(
                center_lat, center_lon, center_alt, self._drone_spacing)
                
        elif self._formation_type == FormationType.LINE_FORMATION:
            return FormationFactory.calculate_line_formation(
                center_lat, center_lon, center_alt, self._drone_spacing)
                
        elif self._formation_type == FormationType.ARROWHEAD_FORMATION:
            return FormationFactory.calculate_arrowhead_formation(
                center_lat, center_lon, center_alt, self._drone_spacing)
                
        else:
            self.get_logger().error(f"Desteklenmeyen formasyon tipi: {self._formation_type}")
            return None
    
    def _send_navigation_command(self, drone_id, target_position):
        """Belirli bir drone'a navigasyon komutu gönderir"""
        if drone_id not in self._action_clients:
            self.get_logger().error(f"Drone {drone_id} için action client bulunamadı")
            return False
            
        client = self._action_clients[drone_id]
        
        # İlk kez navigasyon gönderiyorsak veya hedef pozisyon değiştiyse
        should_send = False
        if drone_id not in self._last_target_positions:
            should_send = True
        else:
            last_target = self._last_target_positions[drone_id]
            # Hedef pozisyon önemli ölçüde değiştiyse (>0.5m)
            if FormationFactory.calculate_distance_meters(
                last_target['lat'], last_target['lon'],
                target_position['lat'], target_position['lon']) > 0.5:
                should_send = True
        
        # Önceki navigasyon tamamlanmamışsa ve hedef değişmediyse gönderme
        if not should_send and drone_id in self._navigation_futures:
            future = self._navigation_futures[drone_id]
            if future and not future.done():
                return False
                
        # Goal oluştur
        goal = NavigateToPoint.Goal()
        goal.target_latitude_deg = float(target_position['lat'])
        goal.target_longitude_deg = float(target_position['lon'])
        goal.target_altitude_amsl = float(target_position['alt'])
        goal.desired_travel_speed_mps = float(self._navigation_speed)
        goal.acceptance_radius_m = float(self._navigation_acceptance_radius)
        
        # Action istek gönder
        if not client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(f"Drone {drone_id} için navigate_to_point action server hazır değil")
            return False
            
        # Hedefi gönder ve future'ı sakla
        future = client.send_goal_async(goal)
        self._navigation_futures[drone_id] = future
        self._last_target_positions[drone_id] = target_position
        
        # Sonuç callback'i ekle
        future.add_done_callback(
            lambda f, d_id=drone_id: self._navigation_response_callback(f, d_id))
            
        return True
        
    def _navigation_response_callback(self, future, drone_id):
        """Navigasyon isteği sonucu callback'i"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn(f"Drone {drone_id} için navigasyon isteği reddedildi")
            # Future'ı temizle
            if drone_id in self._navigation_futures:
                del self._navigation_futures[drone_id]
            return
            
        # Goal kabul edildi, sonuç future'ını izle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, d_id=drone_id: self._navigation_result_callback(f, d_id))
    
    def _navigation_result_callback(self, future, drone_id):
        """Navigasyon sonucu callback'i"""
        result = future.result().result
        
        if result.success:
            # self.get_logger().debug(f"Drone {drone_id} navigasyon başarılı: {result.message}")
            pass
        else:
            # self.get_logger().warn(f"Drone {drone_id} navigasyon başarısız: {result.message}")
            pass
            
        # Future'ı temizle
        if drone_id in self._navigation_futures:
            del self._navigation_futures[drone_id]


def main(args=None):
    rclpy.init(args=args)
    
    try:
        formation_manager = FormationManager()
        
        # MultiThreaded executor kullan (birden fazla callback grubu olduğu için)
        executor = MultiThreadedExecutor()
        executor.add_node(formation_manager)
        
        try:
            # formation_manager.get_logger().debug('Formasyon Yöneticisi çalışıyor. Ctrl+C ile durdurun.')
            executor.spin()
        except KeyboardInterrupt:
            # formation_manager.get_logger().debug('Kullanıcı tarafından durduruldu')
            pass
        finally:
            executor.shutdown()
            formation_manager.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
