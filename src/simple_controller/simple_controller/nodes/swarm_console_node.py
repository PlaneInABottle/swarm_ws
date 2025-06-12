#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType, ParameterValue
from threading import Thread
import os
import sys
import time
import random
import curses
import numpy as np
import sys

# Konsol renkleri için ANSI kodları
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Formation utilities
from ..utils.formation_factory import FormationFactory
from ..utils.random_formation import RandomFormationGenerator

class SwarmConsoleInterface(Node):
    """
    Interactive console interface for Acar Surta Decentralize Swarm UAV control.
    Allows real-time parameter adjustment for formation control and navigation.
    """
    
    def __init__(self):
        super().__init__('swarm_console')
        
        # Default formation parameters
        self.swarm_altitude = 20.0
        self.formation_duration = 30.0
        self.agent_spacing = 8.0
        self.formation_type = "v"
        self.mission_type = "NONE"
        self.takeoff_sent = False
        
        # Initialize a client for formation manager parameter service
        self.formation_manager_set_params = self.create_client(
            SetParameters, '/formation_manager/set_parameters'
        )
        
        # Initialize a client for swarm orchestrator parameter service
        self.orchestrator_set_params = self.create_client(
            SetParameters, '/swarm_orchestrator/set_parameters'
        )
        
        # Wait for services to become available
        while not self.formation_manager_set_params.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for formation manager parameter service...')
            
        while not self.orchestrator_set_params.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for swarm orchestrator parameter service...')
            
        # Default waypoints for navigation mission
        self.waypoints = [
            {'lat': 47.3977548, 'lon': 8.5456534, 'alt': 25},
            {'lat': 47.3979548, 'lon': 8.5455534, 'alt': 25},
            {'lat': 47.3978548, 'lon': 8.5457534, 'alt': 25}
        ]
        
        # Setup parameters
        self.declare_parameter('mission_running', False)
        
        # Service clients for setting parameters on other nodes
        self.formation_manager_set_params = self.create_client(
            SetParameters,
            '/formation_manager/set_parameters'
        )
        
        self.get_logger().info('Swarm Console Interface started')

    def set_formation_parameters(self):
        """Set formation parameters via ROS2 parameter service"""
        # Create formation parameters
        request = SetParameters.Request(
            parameters=[
                Parameter(
                    name='center_altitude',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.swarm_altitude)
                ),
                Parameter(
                    name='duration',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.formation_duration)
                ),
                Parameter(
                    name='agent_spacing',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.agent_spacing)
                ),
                Parameter(
                    name='formation_type',
                    value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=self.formation_type)
                )
            ]
        )
        
        # Call the service
        future = self.formation_manager_set_params.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if all(r.successful for r in response.results):
                self.get_logger().info('Successfully set formation parameters')
                return True
            else:
                self.get_logger().error('Failed to set some parameters')
                return False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False
    
    def send_takeoff_command(self):
        """Orchestrator'a takeoff komutu gönderir"""
        if self.takeoff_sent:
            print(f"{Colors.YELLOW}Takeoff komutu zaten gönderildi!{Colors.END}")
            return False
            
        # Orchestrator'a start_takeoff parametresi gönder
        request = SetParameters.Request(
            parameters=[
                Parameter(
                    name='start_takeoff',
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
                )
            ]
        )
        
        # Call the service
        future = self.orchestrator_set_params.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if all(r.successful for r in response.results):
                self.get_logger().info('Successfully sent takeoff command')
                self.takeoff_sent = True
                print(f"{Colors.GREEN}Takeoff komutu başarıyla gönderildi!{Colors.END}")
                return True
            else:
                self.get_logger().error('Failed to send takeoff command')
                print(f"{Colors.RED}Takeoff komutu gönderilemedi!{Colors.END}")
                return False
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {e}')
            print(f"{Colors.RED}Takeoff servis çağrısı başarısız: {e}{Colors.END}")
            return False
            
    def update_formation(self, formation_type):
        """Formation tipi günceller"""
        # Formasyon yöneticisine parametre gönder
        request = SetParameters.Request(
            parameters=[
                Parameter(
                    name='formation_type',
                    value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=formation_type)
                )
            ]
        )
        
        # Call the service
        future = self.formation_manager_set_params.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if all(r.successful for r in response.results):
                print(f"{Colors.GREEN}Formasyon tipi '{formation_type}' olarak başarıyla güncellendi!{Colors.END}")
                self.formation_type = formation_type
                return True
            else:
                print(f"{Colors.RED}Formasyon tipi güncellenemedi!{Colors.END}")
                return False
        except Exception as e:
            print(f"{Colors.RED}Formasyon güncelleme başarısız: {e}{Colors.END}")
            return False

    def generate_random_formation(self, center_lat, center_lon, center_alt):
        """Generates a random formation around the center point using improved RandomFormationGenerator"""
        # Use the optimized RandomFormationGenerator implemented in the utility module
        # This provides better drone distribution and collision avoidance
        num_drones = 3  # Assuming 3 drones total, can be extended in the future
        
        # Generate formation using RandomFormationGenerator that properly uses swarm_centroid
        positions = RandomFormationGenerator.calculate_random_formation(
            center_lat=center_lat,
            center_lon=center_lon,
            center_alt=center_alt,
            spacing_meters=self.agent_spacing,
            num_drones=num_drones
        )
        
        self.get_logger().info(f'Generated random formation for {num_drones} drones with spacing {self.agent_spacing}m')
        return positions
        
    def navigate_to_waypoints(self):
        """Initiates navigation to predefined waypoints"""
        self.get_logger().info('Starting navigation mission with waypoints')
        # This would interact with the swarm orchestrator to send waypoints
        # Implementation depends on how the swarm orchestrator accepts waypoints
        
    def run_interactive_console(self):
        """Runs the interactive console interface"""
        os.system('clear')
        
        # Başlık ve logo
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╔══════════════════════════════════════════════════════════════════╗{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW} █████╗  ██████╗ █████╗ ██████╗   ███████╗██╗   ██╗██████╗ ████████╗ █████╗  {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW} ██╔══██╗██╔════╝██╔══██╗██╔══██╗  ██╔════╝██║   ██║██╔══██╗╚══██╔══╝██╔══██╗ {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW} ███████║██║     ███████║██████╔╝  ███████╗██║   ██║██████╔╝   ██║   ███████║ {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW} ██╔══██║██║     ██╔══██║██╔══██╗  ╚════██║██║   ██║██╔══██╗   ██║   ██╔══██║ {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW} ██║  ██║╚██████╗██║  ██║██║  ██║  ███████║╚██████╔╝██║  ██║   ██║   ██║  ██║ {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW} ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝  ╚══════╝ ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝ {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╚══════════════════════════════════════════════════════════════════╝{Colors.END}")
        print(f"{Colors.CYAN}{Colors.BOLD}       DECENTRALIZE SWARM UAV CONTROL CONSOLE v1.0       {Colors.END}")
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}")
        
        print(f"\n{Colors.GREEN}Bağlı ROS2 sistemleri kontrol ediliyor...{Colors.END}")
        if self.formation_manager_set_params.wait_for_service(timeout_sec=2.0):
            print(f"{Colors.GREEN}✓ Formation Manager Node aktif ve bağlantı hazır{Colors.END}")
        else:
            print(f"{Colors.RED}✗ Formation Manager Node'a bağlanılamadı!{Colors.END}")
            print(f"{Colors.YELLOW}  Formation Manager Node otomatik başlatılacak{Colors.END}")
        print()
            
        while True:
            print(f"\n{Colors.CYAN}{Colors.BOLD}Sürü UAV Kontrol Konsolu:{Colors.END}")
            print(f"{Colors.CYAN}{'─' * 50}{Colors.END}")
            print(f"{Colors.CYAN}T.{Colors.END} {Colors.BOLD}Takeoff{Colors.END} - Tüm dronları kaldır")
            print(f"{Colors.CYAN}1.{Colors.END} {Colors.BOLD}3B Formasyon Görevi{Colors.END} - Drone sürüsü formasyon kontrolü")
            print(f"{Colors.CYAN}2.{Colors.END} {Colors.BOLD}Navigasyon Görevi{Colors.END} - Waypoint'ler arası sürü hareketi")
            print(f"{Colors.CYAN}F.{Colors.END} {Colors.BOLD}Formasyon Güncelle{Colors.END} - Mevcut formasyonu değiştir")
            print(f"{Colors.CYAN}0.{Colors.END} {Colors.BOLD}Çıkış{Colors.END}")
            
            choice = input(f"\n{Colors.YELLOW}Seçiminiz: {Colors.END}").upper()
            
            if choice == "T":
                print(f"\n{Colors.YELLOW}Takeoff komutu gönderiliyor...{Colors.END}")
                if self.send_takeoff_command():
                    print(f"{Colors.GREEN}Tüm dronlar kalkışa geçiyor!{Colors.END}")
                else:
                    print(f"{Colors.RED}Takeoff komutu başarısız oldu. Tekrar deneyin.{Colors.END}")
            
            elif choice == "F":
                self.update_formation_menu()
                
            elif choice == "1":
                self.mission_type = "3D"
                self.run_3d_formation_menu()
                
            elif choice == "2":
                self.mission_type = "NAV"
                self.run_navigation_menu()
                
            elif choice == "0":
                print(f"\n{Colors.GREEN}Sürü kontrol konsolu kapatılıyor...{Colors.END}")
                break
                
            else:
                print(f"{Colors.RED}Geçersiz seçim. Lütfen tekrar deneyin.{Colors.END}")
    
    def run_3d_formation_menu(self):
        """Runs the 3D formation mission menu"""
        os.system('clear')
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╔══════════════════════════════════════════════════════════════════╗{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW}                   3B FORMASYON GÖREVİ                          {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╚══════════════════════════════════════════════════════════════════╝{Colors.END}")
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}\n")
        
        print(f"{Colors.CYAN}Formasyon Parametreleri{Colors.END}")
        print(f"{Colors.CYAN}{'─' * 30}{Colors.END}")
        
        try:
            alt = float(input(f"{Colors.GREEN}Sürü irtifası giriniz (metre) {Colors.CYAN}[{self.swarm_altitude}]{Colors.END}: "))
            self.swarm_altitude = alt
            print(f"{Colors.BOLD}✓ İrtifa: {self.swarm_altitude}m{Colors.END}")
        except ValueError:
            print(f"{Colors.YELLOW}Geçersiz değer, varsayılan değer kullanılıyor: {self.swarm_altitude} m{Colors.END}")
        
        try:
            duration = float(input(f"\n{Colors.GREEN}Formasyon koruma süresi (saniye) {Colors.CYAN}[{self.formation_duration}]{Colors.END}: "))
            self.formation_duration = duration
            print(f"{Colors.BOLD}✓ Süre: {self.formation_duration}s{Colors.END}")
        except ValueError:
            print(f"{Colors.YELLOW}Geçersiz değer, varsayılan değer kullanılıyor: {self.formation_duration} s{Colors.END}")
        
        try:
            spacing = float(input(f"\n{Colors.GREEN}Ajanlar arası mesafe (metre) {Colors.CYAN}[{self.agent_spacing}]{Colors.END}: "))
            self.agent_spacing = spacing
            print(f"{Colors.BOLD}✓ Mesafe: {self.agent_spacing}m{Colors.END}")
        except ValueError:
            print(f"{Colors.YELLOW}Geçersiz değer, varsayılan değer kullanılıyor: {self.agent_spacing} m{Colors.END}")
        
        print(f"\n{Colors.CYAN}Formasyon Tipi{Colors.END}")
        print(f"{Colors.CYAN}{'─' * 30}{Colors.END}")
        print(f"{Colors.CYAN}1.{Colors.END} V formasyonu        {Colors.YELLOW}〈 Kuş uçuş formasyonu 〉{Colors.END}")
        print(f"{Colors.CYAN}2.{Colors.END} Line formasyonu     {Colors.YELLOW}〈 Düz çizgi formasyonu 〉{Colors.END}")
        print(f"{Colors.CYAN}3.{Colors.END} Arrowhead formasyonu {Colors.YELLOW}〈 Ok başı formasyonu 〉{Colors.END}")
        print(f"{Colors.CYAN}4.{Colors.END} Random formasyon    {Colors.YELLOW}〈 Rastgele dağılım 〉{Colors.END}")
        
        formation_choice = input(f"\n{Colors.GREEN}Formasyon tipi seçin{Colors.END}: ")
        
        if formation_choice == "1":
            self.formation_type = "v"
            formation_name = "V formasyonu"
        elif formation_choice == "2":
            self.formation_type = "line"
            formation_name = "Line formasyonu"
        elif formation_choice == "3":
            self.formation_type = "arrowhead"
            formation_name = "Arrowhead formasyonu"
        elif formation_choice == "4":
            self.formation_type = "random"  
            formation_name = "Random formasyon"
        else:
            print(f"{Colors.YELLOW}Geçersiz seçim, varsayılan V formasyonu kullanılıyor.{Colors.END}")
            self.formation_type = "v"
        
        print(f"\n{Colors.GREEN}Formasyon parametreleri ayarlanıyor...{Colors.END}")
        if self.set_formation_parameters():
            print(f"{Colors.GREEN}Başarılı! {formation_name} ayarlandı.{Colors.END}")
            print(f"{Colors.GREEN}İrtifa: {self.swarm_altitude}m, Süre: {self.formation_duration}s, Mesafe: {self.agent_spacing}m{Colors.END}")
        else:
            print(f"{Colors.RED}Formasyon parametreleri ayarlanamadı!{Colors.END}")
            
        print(f"{Colors.BOLD}\u2713 Se\u00e7ilen formasyon: {Colors.GREEN}{formation_name}{Colors.END}")        
        
        print(f"\n{Colors.GREEN}Formasyon parametreleri ayarlan\u0131yor...{Colors.END}")
        params_set = self.set_formation_parameters()
        
        print(f"\n{Colors.CYAN}Formasyon G\u00f6revi Parametreleri{Colors.END}")
        print(f"{Colors.CYAN}{'\u2500' * 30}{Colors.END}")
        print(f"{Colors.YELLOW}\u0130rtifa:\t{Colors.GREEN}{self.swarm_altitude} m{Colors.END}")
        print(f"{Colors.YELLOW}S\u00fcre:\t\t{Colors.GREEN}{self.formation_duration} s{Colors.END}")
        print(f"{Colors.YELLOW}Mesafe:\t{Colors.GREEN}{self.agent_spacing} m{Colors.END}")
        print(f"{Colors.YELLOW}Formasyon:\t{Colors.GREEN}{formation_name}{Colors.END}")
        
        if params_set:
            print(f"\n{Colors.BOLD}{Colors.GREEN}Formasyon g\u00f6revi ba\u015far\u0131yla ba\u015flat\u0131ld\u0131!{Colors.END}")
            print(f"\nFormasyon {self.formation_duration} saniye korunacak...")
            
            for i in range(int(self.formation_duration), 0, -1):
                sys.stdout.write(f"\rKalan süre: {i} saniye")
                sys.stdout.flush()
                time.sleep(1)
                
            print("\n\nFormasyon tamamlandı!")
        else:
            print("Formasyon parametreleri ayarlanamadı!")
        
        input("\nDevam etmek için Enter tuşuna basın...")
    
    def update_formation_menu(self):
        """Formasyon tipini güncelleme menüsü"""
        os.system('clear')
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╔══════════════════════════════════════════════════════════════════╗{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW}                FORMASYON GÜNCELLEME                           {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╚══════════════════════════════════════════════════════════════════╝{Colors.END}")
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}\n")
        
        print(f"{Colors.CYAN}Mevcut Formasyon: {Colors.YELLOW}{self.formation_type}{Colors.END}")
        print(f"{Colors.CYAN}{'─' * 30}{Colors.END}")
        
        print(f"\n{Colors.CYAN}Formasyon Tipi{Colors.END}")
        print(f"{Colors.CYAN}{'─' * 30}{Colors.END}")
        print(f"{Colors.CYAN}1.{Colors.END} V formasyonu        {Colors.YELLOW}〈 Kuş uçuş formasyonu 〉{Colors.END}")
        print(f"{Colors.CYAN}2.{Colors.END} Line formasyonu     {Colors.YELLOW}〈 Düz çizgi formasyonu 〉{Colors.END}")
        print(f"{Colors.CYAN}3.{Colors.END} Arrowhead formasyonu {Colors.YELLOW}〈 Ok başı formasyonu 〉{Colors.END}")
        print(f"{Colors.CYAN}4.{Colors.END} Random formasyon    {Colors.YELLOW}〈 Rastgele dağılım 〉{Colors.END}")
        print(f"{Colors.CYAN}0.{Colors.END} Geri dön           {Colors.YELLOW}〈 Ana menüye dön 〉{Colors.END}")
        
        formation_choice = input(f"\n{Colors.GREEN}Formasyon tipi seçin{Colors.END}: ")
        
        if formation_choice == "0":
            return
        
        if formation_choice == "1":
            formation_type = "v"
            formation_name = "V formasyonu"
        elif formation_choice == "2":
            formation_type = "line"
            formation_name = "Line formasyonu"
        elif formation_choice == "3":
            formation_type = "arrowhead"
            formation_name = "Arrowhead formasyonu"
        elif formation_choice == "4":
            formation_type = "random"  
            formation_name = "Random formasyon"
        else:
            print(f"{Colors.YELLOW}Geçersiz seçim, formasyon güncelleme işlemi iptal edildi.{Colors.END}")
            input(f"\n{Colors.CYAN}Ana menüye dönmek için ENTER tuşuna basın...{Colors.END}")
            return
            
        print(f"\n{Colors.GREEN}Formasyon güncelleniyor: {formation_name}...{Colors.END}")
        if self.update_formation(formation_type):
            print(f"{Colors.GREEN}Formasyon başarıyla güncellendi!{Colors.END}")
        else:
            print(f"{Colors.RED}Formasyon güncellenemedi!{Colors.END}")
            
        input(f"\n{Colors.CYAN}Ana menüye dönmek için ENTER tuşuna basın...{Colors.END}")
        
    def run_navigation_menu(self):
        """Runs the navigation mission menu"""
        os.system('clear')
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╔══════════════════════════════════════════════════════════════════╗{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW}                  NAVİGASYON GÖREVİ                           {Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╚══════════════════════════════════════════════════════════════════╝{Colors.END}")
        print(f"{Colors.HEADER}{'═' * 70}{Colors.END}\n")
        
        print(f"{Colors.CYAN}Navigasyon Parametreleri{Colors.END}")
        print(f"{Colors.CYAN}{'─' * 30}{Colors.END}")
        
        try:
            alt = float(input(f"{Colors.GREEN}Sürü irtifası giriniz (metre) {Colors.CYAN}[{self.swarm_altitude}]{Colors.END}: "))
            self.swarm_altitude = alt
            print(f"{Colors.BOLD}✓ İrtifa: {self.swarm_altitude}m{Colors.END}")
        except ValueError:
            print(f"{Colors.YELLOW}Geçersiz değer, varsayılan değer kullanılıyor: {self.swarm_altitude} m{Colors.END}")
        
        try:
            duration = float(input(f"\n{Colors.GREEN}Formasyon koruma süresi (saniye) {Colors.CYAN}[{self.formation_duration}]{Colors.END}: "))
            self.formation_duration = duration
            print(f"{Colors.BOLD}✓ Süre: {self.formation_duration}s{Colors.END}")
        except ValueError:
            print(f"{Colors.YELLOW}Geçersiz değer, varsayılan değer kullanılıyor: {self.formation_duration} s{Colors.END}")
        
        try:
            spacing = float(input(f"\n{Colors.GREEN}Ajanlar arası mesafe (metre) {Colors.CYAN}[{self.agent_spacing}]{Colors.END}: "))
            self.agent_spacing = spacing
            print(f"{Colors.BOLD}✓ Mesafe: {self.agent_spacing}m{Colors.END}")
        except ValueError:
            print(f"{Colors.YELLOW}Geçersiz değer, varsayılan değer kullanılıyor: {self.agent_spacing} m{Colors.END}")
        
        print(f"\n{Colors.CYAN}Hareket Düzenini Seçin{Colors.END}")
        print(f"{Colors.CYAN}{'\u2500' * 30}{Colors.END}")
        print(f"{Colors.CYAN}1.{Colors.END} V formasyonu        {Colors.YELLOW}\u3008 Kuş uçuş düzeninde hareket \u3009{Colors.END}")
        print(f"{Colors.CYAN}2.{Colors.END} Line formasyonu     {Colors.YELLOW}\u3008 Düz çizgi düzeninde hareket \u3009{Colors.END}")
        print(f"{Colors.CYAN}3.{Colors.END} Arrowhead formasyonu {Colors.YELLOW}\u3008 Ok başı düzeninde hareket \u3009{Colors.END}")
        print(f"{Colors.CYAN}4.{Colors.END} Random formasyon    {Colors.YELLOW}\u3008 Dağınık düzen \u3009{Colors.END}")
        
        formation_choice = input(f"\n{Colors.GREEN}Düzen seçin{Colors.END}: ")
        
        if formation_choice == "1":
            self.formation_type = "v"
            formation_name = "V formasyonu"
        elif formation_choice == "2":
            self.formation_type = "line"
            formation_name = "Line formasyonu"
        elif formation_choice == "3":
            self.formation_type = "arrowhead"
            formation_name = "Arrowhead formasyonu"
        elif formation_choice == "4":
            self.formation_type = "random"
            formation_name = "Random formasyon"
        else:
            print(f"{Colors.YELLOW}Geçersiz seçim, varsayılan V formasyonu kullanılıyor.{Colors.END}")
            self.formation_type = "v"
            formation_name = "V formasyonu"
            
        print(f"{Colors.BOLD}\u2713 Seçilen düzen: {Colors.GREEN}{formation_name}{Colors.END}")
        
        # Display default waypoints
        print(f"\n{Colors.CYAN}Navigasyon Hedef Noktaları (GPS){Colors.END}")
        print(f"{Colors.CYAN}{'\u2500' * 30}{Colors.END}")
        for i, wp in enumerate(self.waypoints):
            print(f"{Colors.YELLOW}Nokta {i+1}:{Colors.END} Lat: {Colors.BOLD}{wp['lat']}{Colors.END}, Lon: {Colors.BOLD}{wp['lon']}{Colors.END}, Alt: {Colors.BOLD}{wp['alt']}m{Colors.END}")
        
        print(f"\n{Colors.GREEN}Navigasyon parametreleri ayarlanıyor...{Colors.END}")
        params_set = self.set_formation_parameters()
        
        print(f"\n{Colors.CYAN}Navigasyon Görev Özeti{Colors.END}")
        print(f"{Colors.CYAN}{'\u2500' * 30}{Colors.END}")
        print(f"{Colors.YELLOW}İrtifa:\t{Colors.GREEN}{self.swarm_altitude} m{Colors.END}")
        print(f"{Colors.YELLOW}Düzen:\t{Colors.GREEN}{formation_name}{Colors.END}")
        print(f"{Colors.YELLOW}Mesafe:\t{Colors.GREEN}{self.agent_spacing} m{Colors.END}")
        print(f"{Colors.YELLOW}Hedefler:\t{Colors.GREEN}{len(self.waypoints)} nokta{Colors.END}")
        
        if params_set:
            print(f"\n{Colors.BOLD}{Colors.GREEN}Navigasyon görevi başarıyla başlatıldı!{Colors.END}")
            print(f"\n{Colors.CYAN}Drone sürüsü hedeflere yöneliyor...{Colors.END}")
            self.navigate_to_waypoints()
            print(f"\n{Colors.GREEN}Tüm hedefler başarıyla tamamlandı!{Colors.END}")
        else:
            print(f"\n{Colors.RED}Navigasyon parametreleri ayarlanamadı! Formation Manager node kontrol edin.{Colors.END}")
        
        input(f"\n{Colors.YELLOW}Ana menüye dönmek için Enter tuşuna basın...{Colors.END}")


def main(args=None):
    """Main function for the swarm console node"""
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create the console interface node
    console = SwarmConsoleInterface()
    
    # Create and start a thread for ROS processing
    ros_thread = Thread(target=lambda: rclpy.spin(console))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Run the interactive console
        console.run_interactive_console()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Clean up
        console.destroy_node()
        rclpy.shutdown()
        
    print("Console terminated.")


if __name__ == '__main__':
    main()
