#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import threading
import asyncio
import sys
import os
from typing import Dict, Any, List
import shutil  # For getting terminal size

# Konsol renkleri için ANSI kodları
class Colors:
    """Terminalde renkli metinler için ANSI kaçış kodları."""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class FormationConsole(Node):
    """
    Formasyon kontrol konsolu node'u.
    Kullanıcı arayüzü üzerinden sürü formasyonunu kontrol etmeyi sağlar.
    """

    def __init__(self):
        """Node'u ve gerekli istemcileri (clients) başlatır."""
        super().__init__('formation_console')
        
        # Formation manager node ismi
        self.formation_manager_name = "/formation_manager"
        
        # Set parameters client
        self.set_param_client = self.create_client(
            SetParameters, 
            f'{self.formation_manager_name}/set_parameters'
        )
        
        # Formasyon koruma süresi için parametre tanımla
        self.declare_parameter('formation_hold_time', 0.0)
        
        # Wait for service to be available
        self.get_logger().info("Formation Konsolu başlatılıyor. Servis bağlantısı kuruluyor...")
        if not self.set_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{Colors.RED}HATA: '{self.formation_manager_name}' servisi bulunamadı!{Colors.END}")
        else:
            self.get_logger().info(f"{Colors.GREEN}✓ '{self.formation_manager_name}' servisine başarıyla bağlanıldı!{Colors.END}")
            
        # Main menu selections
        self.menu_options = {
            "1": self._update_formation_type,
            "2": self._update_drone_spacing,
            "3": self._update_formation_duration,
            "4": self._enable_disable_formation,
            "5": self._update_navigation_speed,
            "6": self._update_acceptance_radius,
            "7": self._move_forward_control,
            "8": self._update_forward_distance,
            "q": self._quit
        }
        
        # Supported formation types
        self.formation_types = ["v", "line", "arrowhead", "random"]
        
        # Flag for main loop
        self.should_exit = False
        
    def _create_parameter(self, name: str, value: Any) -> Parameter:
        """(Kullanımda değil, referans için) Parameter mesajı oluşturur."""
        param_msg = Parameter()
        param_msg.name = name
        
        param_value = ParameterValue()
        
        if isinstance(value, str):
            param_value.type = ParameterType.PARAMETER_STRING
            param_value.string_value = value
        elif isinstance(value, bool):
            param_value.type = ParameterType.PARAMETER_BOOL
            param_value.bool_value = value
        elif isinstance(value, int):
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = value
        elif isinstance(value, float):
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = value
        else:
            self.get_logger().error(f"Desteklenmeyen parametre tipi: {type(value)}")
            return None
            
        param_msg.value = param_value
        return param_msg
        
    async def _set_parameter(self, name: str, value: Any) -> bool:
        """Formation Manager node'una parametre değeri gönderir."""
        if not self.set_param_client.service_is_ready():
            self.get_logger().error("Formation Manager servisi hazır değil!")
            return False
            
        request = SetParameters.Request()
        param = Parameter()
        param.name = name
        
        if isinstance(value, str):
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        elif isinstance(value, bool):
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        elif isinstance(value, int):
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = value
        elif isinstance(value, float):
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
        else:
            self.get_logger().error(f"Desteklenmeyen parametre tipi: {type(value)}")
            return False
        
        request.parameters = [param]
        self.get_logger().info(f"Parametre güncelleniyor: {name} = {value}")
        
        try:
            future = self.set_param_client.call_async(request)
            await future
            
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info(f"Parametre başarıyla güncellendi: {name} = {value}")
                return True
            else:
                self.get_logger().error(f"Parametre güncellenemedi: {response.results[0].reason}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Parametre güncelleme hatası: {str(e)}")
            return False
            
    def _clear_screen(self):
        """Platforma uygun olarak konsol ekranını temizler."""
        os.system('cls' if os.name == 'nt' else 'clear')
        
    def _print_section_header(self, title: str):
        """Menü bölümleri için standart bir başlık yazdırır."""
        term_width = shutil.get_terminal_size().columns
        print(f"{Colors.HEADER}{'═' * term_width}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╔{'═' * (term_width - 2)}╗{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW}{title.center(term_width - 2)}{Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╚{'═' * (term_width - 2)}╝{Colors.END}")
        print(f"{Colors.HEADER}{'═' * term_width}{Colors.END}\n")
        
    def _print_header(self):
        """Konsol için ana başlığı ve ASCII art logoyu yazdırır."""
        self._clear_screen()
        term_width = shutil.get_terminal_size().columns
        
        logo = [
            " █████╗  ██████╗ █████╗ ██████╗   ███████╗██╗   ██╗██████╗ ████████╗ █████╗ ",
            " ██╔══██╗██╔════╝██╔══██╗██╔══██╗  ██╔════╝██║   ██║██╔══██╗╚══██╔══╝██╔══██╗",
            " ███████║██║     ███████║██████╔╝  ███████╗██║   ██║██████╔╝   ██║   ███████║",
            " ██╔══██║██║     ██╔══██║██╔══██╗  ╚════██║██║   ██║██╔══██╗   ██║   ██╔══██║",
            " ██║  ██║╚██████╗██║  ██║██║  ██║  ███████║╚██████╔╝██║  ██║   ██║   ██║  ██║",
            " ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝  ╚══════╝ ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝"
        ]
        
        print(f"\n{Colors.HEADER}{'═' * term_width}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╔{'═' * (term_width - 2)}╗{Colors.END}")
        for line in logo:
            # Logoyu terminal genişliğine sığdırmak için kontrol
            if len(line) < term_width - 4:
                print(f"{Colors.BOLD}{Colors.BLUE}║{Colors.YELLOW}{line.center(term_width - 2)}{Colors.BLUE}║{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}╚{'═' * (term_width - 2)}╝{Colors.END}")
        
        subtitle = "DECENTRALIZE SWARM UAV CONTROL CONSOLE"
        print(f"{Colors.CYAN}{Colors.BOLD}{subtitle.center(term_width)}{Colors.END}")
        
        version = "v1.0.0"
        print(f"{version.rjust(term_width)}")
        
        print(f"{Colors.HEADER}{'═' * term_width}{Colors.END}\n")
        
    def _print_menu(self):
        """Ana menüyü seçeneklerle birlikte yazdırır."""
        self._print_header()
        term_width = shutil.get_terminal_size().columns
        
        print(f"{Colors.CYAN}{Colors.BOLD}Sürü UAV Kontrol Konsolu:{Colors.END}")
        print(f"{Colors.CYAN}{'─' * term_width}{Colors.END}")
        
        print(f"{Colors.CYAN}1.{Colors.END} {Colors.BOLD}Formasyon Tipini Değiştir{Colors.END} {Colors.YELLOW}〈 V, Line, Arrowhead , Random〉{Colors.END}")
        print(f"{Colors.CYAN}2.{Colors.END} {Colors.BOLD}Drone'lar Arası Mesafeyi Güncelle{Colors.END} {Colors.YELLOW}〈 Metre cinsinden 〉{Colors.END}")
        print(f"{Colors.CYAN}3.{Colors.END} {Colors.BOLD}Formasyon Koruma Süresini Güncelle{Colors.END} {Colors.YELLOW}〈 Saniye, 0=Sürekli 〉{Colors.END}")
        print(f"{Colors.CYAN}4.{Colors.END} {Colors.BOLD}Formasyonu Aktif/Pasif Yap{Colors.END} {Colors.YELLOW}〈 Formasyon görevini başlat/durdur 〉{Colors.END}")
        print(f"{Colors.CYAN}5.{Colors.END} {Colors.BOLD}Navigasyon Hızını Güncelle{Colors.END} {Colors.YELLOW}〈 m/s cinsinden 〉{Colors.END}")
        # print(f"{Colors.CYAN}6.{Colors.END} {Colors.BOLD}Hedef Kabul Yarıçapını Güncelle{Colors.END} {Colors.YELLOW}〈 Metre cinsinden 〉{Colors.END}")
        print(f"{Colors.CYAN}7.{Colors.END} {Colors.BOLD}İleri Hareket Başlat{Colors.END} {Colors.YELLOW}〈 Sürü formasyonunu koruyarak ileri git 〉{Colors.END}")
        print(f"{Colors.CYAN}8.{Colors.END} {Colors.BOLD}İleri Hareket Mesafesini Ayarla{Colors.END} {Colors.YELLOW}〈 Metre cinsinden 〉{Colors.END}")
        print(f"{Colors.CYAN}8.{Colors.END} {Colors.BOLD}Waypoint {Colors.END} {Colors.YELLOW}〈 Metre cinsinden 〉{Colors.END}")
        print(f"{Colors.RED}q.{Colors.END} {Colors.BOLD}Çıkış{Colors.END}")
        
        # Dinamik sistem durumu göstergesi
        if self.set_param_client.service_is_ready():
            status = f"{Colors.GREEN}✓ Sistem Durumu: {Colors.BOLD}Bağlı{Colors.END}"
        else:
            status = f"{Colors.RED}✗ Sistem Durumu: {Colors.BOLD}Bağlantı Yok{Colors.END}"
        print(f"\n{status}")
        
    def _get_menu_choice(self) -> str:
        """Kullanıcıdan menü seçimi yapmasını ister."""
        choice = input(f"\n{Colors.YELLOW}{Colors.BOLD}Seçiminiz: {Colors.END}").lower()
        return choice
        
    def _wait_for_enter(self):
        """Kullanıcının devam etmek için Enter'a basmasını bekler."""
        input(f"\n{Colors.CYAN}Ana menüye dönmek için Enter tuşuna basın...{Colors.END}")
        
    async def _update_formation_type(self):
        """Formasyon tipini günceller."""
        self._print_section_header("FORMASYON TİPİ AYARLARI")
        
        print(f"{Colors.CYAN}Mevcut formasyon tipleri:{Colors.END}")
        print(f"{Colors.CYAN}{'─' * 30}{Colors.END}")
        
        for i, formation_type in enumerate(self.formation_types, 1):
            print(f"{Colors.CYAN}{i}.{Colors.END} {Colors.BOLD}[⬤] {formation_type.upper()}{Colors.END} {Colors.YELLOW}〈 Formasyon tipi {i} 〉{Colors.END}")
        
        try:
            choice = int(input(f"\n{Colors.GREEN}Seçiminiz (1-{len(self.formation_types)}): {Colors.END}"))
            if 1 <= choice <= len(self.formation_types):
                formation_type = self.formation_types[choice-1]
                print(f"\n{Colors.YELLOW}⌛ Formasyon güncelleniyor...{Colors.END}")
                success = await self._set_parameter("formation_type", formation_type)
                
                if success:
                    print(f"{Colors.GREEN}✔ Formasyon tipi '{formation_type.upper()}' olarak güncellendi.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Formasyon güncellenemedi!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Geçersiz seçim!{Colors.END}")  
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()
        
    async def _update_drone_spacing(self):
        """Drone'lar arası mesafeyi günceller."""
        self._print_section_header("DRONE'LAR ARASI MESAFE AYARLARI")
        
        print(f"{Colors.YELLOW}• Drone'lar arasındaki mesafeyi ayarlayın{Colors.END}")
        print(f"{Colors.YELLOW}• Mesafe metre cinsinden belirtilmelidir{Colors.END}")
        
        try:
            spacing = float(input(f"\n{Colors.GREEN}Yeni mesafe (metre): {Colors.END}"))
            if spacing > 0:
                print(f"\n{Colors.YELLOW}⌛ Mesafe güncelleniyor...{Colors.END}")
                success = await self._set_parameter("drone_spacing", spacing)
                
                if success:
                    print(f"{Colors.GREEN}✔ Drone'lar arası mesafe {spacing:.2f} metre olarak güncellendi.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Drone'lar arası mesafe güncellenemedi!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Mesafe pozitif bir değer olmalıdır!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()
        
    async def _update_formation_duration(self):
        """Formasyon koruma süresini günceller."""
        self._print_section_header("FORMASYON KORUMA SÜRESİ AYARLARI")
        
        print(f"{Colors.YELLOW}• Formasyon koruma süresi, dronların belirlenen formasyonda kalacağı süreyi belirtir{Colors.END}")
        print(f"{Colors.YELLOW}• 0 değeri dronların sürekli formasyonda kalacağını belirtir{Colors.END}")
        
        try:
            duration = float(input(f"\n{Colors.GREEN}Formasyon koruma süresi (saniye, 0=sürekli): {Colors.END}"))
            if duration >= 0:
                print(f"\n{Colors.YELLOW}⌛ Süre güncelleniyor...{Colors.END}")
                success = await self._set_parameter("formation_hold_time", duration)
                
                if success:
                    if duration == 0:
                        print(f"{Colors.GREEN}✔ Formasyon koruma süresi {Colors.BOLD}SÜREKLİ{Colors.END}{Colors.GREEN} olarak ayarlandı.{Colors.END}")
                    else:
                        print(f"{Colors.GREEN}✔ Formasyon koruma süresi {duration:.1f} saniye olarak ayarlandı.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Formasyon koruma süresi ayarlanamadı!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Süre 0 veya pozitif bir değer olmalıdır!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()
        
    async def _enable_disable_formation(self):
        """Formasyonu aktif/pasif yapar."""
        self._print_section_header("FORMASYON AKTİFLEŞTİRME KONTROLÜ")
        
        print(f"{Colors.YELLOW}• Formasyon durumunu seçin:{Colors.END}")
        print("")
        print(f"   {Colors.GREEN}1.{Colors.END} {Colors.BOLD}⬤ FORMASYON AKTİF{Colors.END}")
        print(f"   {Colors.RED}2.{Colors.END} {Colors.BOLD}⭕ FORMASYON PASİF{Colors.END}")
        
        try:
            choice = int(input(f"\n{Colors.GREEN}Seçiminiz (1-2): {Colors.END}"))
            if choice == 1:
                print(f"\n{Colors.YELLOW}⌛ Formasyon aktifleştiriliyor...{Colors.END}")
                success = await self._set_parameter("enable_formation", True)
                
                if success:
                    print(f"{Colors.GREEN}✔ Formasyon başarıyla {Colors.BOLD}AKTİFLEŞTİRİLDİ.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Formasyon durumu güncellenemedi!{Colors.END}")
            elif choice == 2:
                print(f"\n{Colors.YELLOW}⌛ Formasyon pasifleştiriliyor...{Colors.END}")
                success = await self._set_parameter("enable_formation", False)
                
                if success:
                    print(f"{Colors.GREEN}✔ Formasyon başarıyla {Colors.BOLD}PASİFLEŞTİRİLDİ.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Formasyon durumu güncellenemedi!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Geçersiz seçim!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()

    async def _update_navigation_speed(self):
        """Navigasyon hızını günceller."""
        self._print_section_header("NAVİGASYON HIZI AYARLARI")
        
        print(f"{Colors.YELLOW}• Dronların hedef noktalara giderken kullandığı hızı ayarlayın{Colors.END}")
        print(f"{Colors.YELLOW}• Hız metre/saniye cinsinden belirtilmelidir{Colors.END}")
        
        try:
            speed = float(input(f"\n{Colors.GREEN}Yeni hız (m/s): {Colors.END}"))
            if speed > 0:
                print(f"\n{Colors.YELLOW}⌛ Hız güncelleniyor...{Colors.END}")
                success = await self._set_parameter("navigation_speed", speed)
                
                if success:
                    print(f"{Colors.GREEN}✔ Navigasyon hızı {speed:.2f} m/s olarak güncellendi.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Navigasyon hızı güncellenemedi!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Hız pozitif bir değer olmalıdır!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()

    async def _update_acceptance_radius(self):
        """Hedef kabul yarıçapını günceller."""
        self._print_section_header("HEDEF KABUL YARIÇAPI AYARLARI")
        
        print(f"{Colors.YELLOW}• Hedef kabul yarıçapı, drone'un hedefe 'ulaştım' kabul edeceği mesafeyi belirler{Colors.END}")
        print(f"{Colors.YELLOW}• Daha küçük değerler daha hassas konumlandırma sağlar{Colors.END}")
        
        try:
            radius = float(input(f"\n{Colors.GREEN}Yeni yarıçap (metre): {Colors.END}"))
            if radius > 0:
                print(f"\n{Colors.YELLOW}⌛ Yarıçap güncelleniyor...{Colors.END}")
                success = await self._set_parameter("navigation_acceptance_radius", radius)
                
                if success:
                    print(f"{Colors.GREEN}✔ Hedef kabul yarıçapı {radius:.2f} metre olarak güncellendi.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ Hedef kabul yarıçapı güncellenemedi!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Yarıçap pozitif bir değer olmalıdır!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()
        
    async def _move_forward_control(self):
        """Sürü formasyonunu koruyarak ileri hareket başlatır."""
        self._print_section_header("SÜRÜ İLERİ HAREKET KONTROLÜ")
        
        print(f"{Colors.YELLOW}• İleri hareket durumunu seçin:{Colors.END}")
        print("")
        print(f"   {Colors.GREEN}1.{Colors.END} {Colors.BOLD}⬤ İLERİ HAREKET BAŞLAT{Colors.END}")
        print(f"   {Colors.RED}2.{Colors.END} {Colors.BOLD}⭕ İLERİ HAREKETİ DURDUR{Colors.END}")
        
        try:
            choice = int(input(f"\n{Colors.GREEN}Seçiminiz (1-2): {Colors.END}"))
            if choice == 1:
                print(f"\n{Colors.YELLOW}⌛ İleri hareket başlatılıyor...{Colors.END}")
                success = await self._set_parameter("move_forward", True)
                
                if success:
                    print(f"{Colors.GREEN}✔ İleri hareket başarıyla {Colors.BOLD}BAŞLATILDI.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ İleri hareket başlatılamadı!{Colors.END}")
            elif choice == 2:
                print(f"\n{Colors.YELLOW}⌛ İleri hareket durduruluyor...{Colors.END}")
                success = await self._set_parameter("move_forward", False)
                
                if success:
                    print(f"{Colors.GREEN}✔ İleri hareket başarıyla {Colors.BOLD}DURDURULDU.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ İleri hareket durdurulamadı!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Geçersiz seçim!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()
    
    async def _update_forward_distance(self):
        """İleri hareket mesafesini günceller."""
        self._print_section_header("İLERİ HAREKET MESAFESİ AYARLARI")
        
        print(f"{Colors.YELLOW}• Sürünün formasyonu koruyarak ileri gideceği mesafeyi ayarlayın{Colors.END}")
        print(f"{Colors.YELLOW}• Mesafe metre cinsinden belirtilmelidir{Colors.END}")
        
        try:
            distance = float(input(f"\n{Colors.GREEN}Yeni ileri hareket mesafesi (metre): {Colors.END}"))
            if distance > 0:
                print(f"\n{Colors.YELLOW}⌛ İleri hareket mesafesi güncelleniyor...{Colors.END}")
                success = await self._set_parameter("forward_distance", distance)
                
                if success:
                    print(f"{Colors.GREEN}✔ İleri hareket mesafesi {distance:.2f} metre olarak güncellendi.{Colors.END}")
                else:
                    print(f"{Colors.RED}✘ İleri hareket mesafesi güncellenemedi!{Colors.END}")
            else:
                print(f"\n{Colors.RED}⚠ Mesafe pozitif bir değer olmalıdır!{Colors.END}")
        except ValueError:
            print(f"\n{Colors.RED}⚠ Lütfen geçerli bir sayı girin!{Colors.END}")
            
        self._wait_for_enter()
        
    async def _quit(self):
        """Programdan çıkış yapar."""
        self._print_header()
        print(f"{Colors.GREEN}Programdan çıkılıyor... Hoşça kalın!{Colors.END}\n")
        self.should_exit = True
        
    async def run_console(self):
        """Ana konsol döngüsü."""
        while not self.should_exit:
            self._print_menu() 
            choice = self._get_menu_choice()
            
            action = self.menu_options.get(choice)
            if action:
                # Menü aksiyonuna geçmeden önce ekranı temizleyip başlığı tekrar basmak
                # daha temiz bir kullanıcı deneyimi sunar.
                self._clear_screen()
                self._print_header()
                await action()
            else:
                print(f"\n{Colors.RED}Geçersiz seçenek! Lütfen tekrar deneyin.{Colors.END}")
                self._wait_for_enter()

def main(args=None):
    """Ana fonksiyon. ROS2'yi başlatır, node'u oluşturur ve konsolu çalıştırır."""
    rclpy.init(args=args)
    
    console = FormationConsole()
    
    # ROS işlemlerini ayrı bir thread'de çalıştırarak UI'ın bloklanmasını önle
    ros_thread = threading.Thread(target=rclpy.spin, args=(console,), daemon=True)
    ros_thread.start()
    
    try:
        # Ana thread'de asyncio ile konsol arayüzünü çalıştır
        asyncio.run(console.run_console())
    except KeyboardInterrupt:
        print("\nKullanıcı tarafından çıkış istendi. Kapatılıyor...")
    finally:
        # Kaynakları temizle ve node'u düzgünce kapat
        console.destroy_node()
        rclpy.shutdown()
        ros_thread.join() # Thread'in düzgünce kapanmasını bekle
        

if __name__ == '__main__':
    main()