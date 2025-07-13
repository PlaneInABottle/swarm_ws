#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import random
from scipy.optimize import linear_sum_assignment
from typing import List, Dict, Any, Tuple, Optional


class FormationFactory:
    """
    Drone sürüleri için farklı formasyon stratejilerini hesaplar.
    - V Formasyonu: Merkez nokta öndeki drone, diğer dronlar arkada V şeklinde
    - Line Formasyonu: Dronlar aynı hizada yan yana dizilirler
    - Arrowhead Formasyonu: Ok başını andıran formasyon (liderin arkasında iki drone)
    - Random Formasyonu: Dronların merkez etrafında çakışma olmayacak şekilde rastgele dağılımı
    """
    
    # WGS84 elipsoid parametreleri
    EARTH_RADIUS_M = 6371000  # Dünya yarıçapı (metre)
    WGS84_A = 6378137.0       # Büyük yarı eksen (metre)
    WGS84_F = 1/298.257223563 # Elipsoid düzleşme parametresi

    @staticmethod
    def calculate_gps_offset_geodesic(lat, lon, north_meters, east_meters) -> Tuple[float, float]:
        """
        Belirli bir GPS noktasından kuzey ve doğu yönündeki ofsetleri jeodezik olarak hesaplar.
        Haversine formülü ve küresel trigonometri kullanılarak daha doğru sonuçlar verir.
        
        Args:
            lat: Başlangıç enlemi (derece)
            lon: Başlangıç boylamı (derece)
            north_meters: Kuzeye doğru mesafe (metre, + kuzey, - güney)
            east_meters: Doğuya doğru mesafe (metre, + doğu, - batı)
            
        Returns:
            (yeni_enlem, yeni_boylam): Hesaplanan GPS koordinatları
        """
        # Radyana çevirme
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # Yerküre üzerindeki açısal mesafeyi hesapla (radyan cinsinden)
        # Kuzey mesafeyi enleme çevir
        delta_lat = north_meters / FormationFactory.EARTH_RADIUS_M
        # Doğu mesafeyi boylamda değişime çevir (enlem faktörünü hesaba katar)
        delta_lon = east_meters / (FormationFactory.EARTH_RADIUS_M * math.cos(lat_rad))
        
        # Yeni koordinatlar (radyan cinsinden)
        new_lat_rad = lat_rad + delta_lat
        new_lon_rad = lon_rad + delta_lon
        
        # Derecelere çevir
        new_lat = math.degrees(new_lat_rad)
        new_lon = math.degrees(new_lon_rad)
        
        return new_lat, new_lon
        
    @staticmethod
    def calculate_gps_offset(lat, lon, north_meters, east_meters) -> Tuple[float, float]:
        """
        Jeodezik hesaplama metoduna geçiş için wrapper fonksiyonu.
        Eski kodu bozmadan yeni metodu kullanmak için.
        """
        return FormationFactory.calculate_gps_offset_geodesic(lat, lon, north_meters, east_meters)
    
    @staticmethod
    def calculate_distance_haversine(lat1, lon1, lat2, lon2) -> float:
        """
        İki GPS noktası arasındaki mesafeyi Haversine formülü kullanarak metre cinsinden hesaplar.
        Bu metod, dünyanın küresel yapısını dikkate alır ve her mesafe için daha doğru sonuç verir.
        
        Args:
            lat1, lon1: Birinci GPS noktası (derece)
            lat2, lon2: İkinci GPS noktası (derece)
            
        Returns:
            float: İki nokta arası mesafe (metre)
        """
        # Radyana çevirme
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Enlem ve boylam farkları
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formülü
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = FormationFactory.EARTH_RADIUS_M * c
        
        return distance
        
    @staticmethod
    def calculate_distance_meters(lat1, lon1, lat2, lon2) -> float:
        """
        İki GPS noktası arasındaki mesafeyi metre cinsinden hesaplar.
        Haversine metoduna geçiş için wrapper fonksiyonu.
        
        Args:
            lat1, lon1: Birinci GPS noktası
            lat2, lon2: İkinci GPS noktası
            
        Returns:
            float: İki nokta arası mesafe (metre)
        """
        return FormationFactory.calculate_distance_haversine(lat1, lon1, lat2, lon2)
    
    @staticmethod
    def calculate_v_formation(center_lat, center_lon, center_alt, spacing_meters=5.0) -> List[Dict[str, float]]:
        """
        V formasyonu için GPS pozisyonlarını hesaplar.
        
        Args:
            center_lat: Merkez enlem
            center_lon: Merkez boylam
            center_alt: İrtifa (m)
            spacing_meters: Dronlar arası mesafe (m)
            
        Returns:
            List[Dict[str, float]]: Her drone için GPS konumları listesi
        """
        positions = []
        
        # Lider drone (merkez, ön)
        positions.append({
            'lat': center_lat,
            'lon': center_lon,
            'alt': center_alt
        })
        
        # Sağ kanat - 120 derece açıyla (gerçek dünyada açı, güney=0, batı=270 dereceye göre)
        # Burada açıları kuzey=0, doğu=90 şeklinde hesaplıyoruz, bu yüzden 120 yerine 210 kullanıyoruz
        angle_right = math.radians(210)
        north_offset_right = spacing_meters * math.cos(angle_right)  # -X (güney)
        east_offset_right = spacing_meters * math.sin(angle_right)   # -Y (batı)
        right_lat, right_lon = FormationFactory.calculate_gps_offset(
            center_lat, center_lon, north_offset_right, east_offset_right)
        
        positions.append({
            'lat': right_lat,
            'lon': right_lon,
            'alt': center_alt
        })
        
        # Sol kanat - 240 derece açıyla (gerçek dünyada)
        angle_left = math.radians(150)  # 360-210 = 150
        north_offset_left = spacing_meters * math.cos(angle_left)  # -X (güney)
        east_offset_left = spacing_meters * math.sin(angle_left)   # +Y (doğu)
        left_lat, left_lon = FormationFactory.calculate_gps_offset(
            center_lat, center_lon, north_offset_left, east_offset_left)
        
        positions.append({
            'lat': left_lat,
            'lon': left_lon,
            'alt': center_alt
        })
        
        return positions
    
    @staticmethod
    def calculate_line_formation(center_lat, center_lon, center_alt, spacing_meters=5.0) -> List[Dict[str, float]]:
        """
        Line (çizgi) formasyonu için GPS pozisyonlarını hesaplar.
        
        Args:
            center_lat: Merkez enlem
            center_lon: Merkez boylam
            center_alt: İrtifa (m)
            spacing_meters: Dronlar arası mesafe (m)
            
        Returns:
            List[Dict[str, float]]: Her drone için GPS konumları listesi
        """
        positions = []
        
        # Orta drone (merkez noktada)
        positions.append({
            'lat': center_lat,
            'lon': center_lon,
            'alt': center_alt
        })
        
        # Sağdaki drone (doğu)
        right_lat, right_lon = FormationFactory.calculate_gps_offset(
            center_lat, center_lon, 0, spacing_meters)
        
        positions.append({
            'lat': right_lat,
            'lon': right_lon,
            'alt': center_alt
        })
        
        # Soldaki drone (batı)
        left_lat, left_lon = FormationFactory.calculate_gps_offset(
            center_lat, center_lon, 0, -spacing_meters)
        
        positions.append({
            'lat': left_lat,
            'lon': left_lon,
            'alt': center_alt
        })
        
        return positions
    
    @staticmethod
    def calculate_arrowhead_formation(center_lat, center_lon, center_alt, spacing_meters=5.0) -> List[Dict[str, float]]:
        """
        Arrowhead (ok başı) formasyonu için GPS pozisyonlarını hesaplar.
        
        Args:
            center_lat: Merkez enlem
            center_lon: Merkez boylam
            center_alt: İrtifa (m)
            spacing_meters: Dronlar arası mesafe (m)
            
        Returns:
            List[Dict[str, float]]: Her drone için GPS konumları listesi
        """
        positions = []
        
        # Lider drone (merkez, ön)
        positions.append({
            'lat': center_lat,
            'lon': center_lon,
            'alt': center_alt
        })
        
        # Sağ kanat - daha dar açı (30°)
        angle_right = math.radians(210)
        north_offset_right = spacing_meters * math.cos(angle_right)  # -X (güney)
        east_offset_right = spacing_meters * math.sin(angle_right)   # -Y (batı)
        right_lat, right_lon = FormationFactory.calculate_gps_offset(
            center_lat, center_lon, north_offset_right, east_offset_right)
        
        positions.append({
            'lat': right_lat,
            'lon': right_lon,
            'alt': center_alt
        })
        
        # Sol kanat - daha dar açı (30°)
        angle_left = math.radians(150)
        north_offset_left = spacing_meters * math.cos(angle_left)  # -X (güney)
        east_offset_left = spacing_meters * math.sin(angle_left)   # +Y (doğu)
        left_lat, left_lon = FormationFactory.calculate_gps_offset(
            center_lat, center_lon, north_offset_left, east_offset_left)
        
        positions.append({
            'lat': left_lat,
            'lon': left_lon,
            'alt': center_alt
        })
        
        return positions
    
    @staticmethod
    def calculate_random_formation(center_lat, center_lon, center_alt, spacing_meters=5.0,
                                num_drones=3) -> List[Dict[str, float]]:
        """
        Rastgele formasyon için GPS pozisyonlarını hesaplar.
        Sürü merkezi (swarm_centroid) etrafında homojen dağılımlı rastgele pozisyonlar oluşturur.
        
        Args:
            center_lat: Merkez enlem (sürü merkezi)
            center_lon: Merkez boylam (sürü merkezi)
            center_alt: İrtifa (m)
            spacing_meters: Dronlar arası minimum mesafe (m)
            num_drones: Toplam drone sayısı
            
        Returns:
            List[Dict[str, float]]: Her drone için GPS konumları listesi
        """
        positions = []
        
        # Lider drone (merkez noktada - swarm centroid'da konumlandırılır)
        positions.append({
            'lat': center_lat,
            'lon': center_lon,
            'alt': center_alt
        })
        
        # Her drone için çakışma olmayan pozisyonlar oluştur
        # Çakışma kontrolü için minimum mesafe eşiği
        min_distance_threshold = spacing_meters * 0.8  # %80 mesafe eşiği
        existing_positions = [(center_lat, center_lon)]  # Lider drone pozisyonu
        
        # Diğer dronlar için rastgele pozisyonlar oluştur
        for i in range(1, num_drones):
            # Her drone için 10 deneme yapalım, en iyi pozisyonu seçelim
            best_position = None
            max_min_distance = 0
            
            for _ in range(10):  # 10 farklı aday pozisyon deneyelim
                # Eşit dağılımlı rastgele açı ve mesafe
                angle = random.uniform(0, 2 * math.pi)
                
                # Dağılımı iyileştirmek için mesafe aralığını drone sayısına göre ayarla
                # Daha fazla drone olduğunda daha geniş bir dağılım için mesafe aralığını artır
                min_spacing = spacing_meters
                max_spacing = spacing_meters * (1.0 + (num_drones / 10.0))  # Drone sayısı arttıkça dağılım genişler
                
                distance = random.uniform(min_spacing, max_spacing)
                
                # Açı ve mesafeyi kuzey-doğu ofsetlerine çevir
                north_offset = distance * math.cos(angle)
                east_offset = distance * math.sin(angle)
                
                # GPS koordinatlarını hesapla
                new_lat, new_lon = FormationFactory.calculate_gps_offset(
                    center_lat, center_lon, north_offset, east_offset)
                
                # Bu pozisyonun diğer dronlarla mesafesini kontrol et
                min_dist = float('inf')
                for lat, lon in existing_positions:
                    dist = FormationFactory.calculate_distance_meters(lat, lon, new_lat, new_lon)
                    min_dist = min(min_dist, dist)
                
                # Eğer bu pozisyon daha iyi bir minimum mesafe sağlıyorsa kaydet
                if min_dist > max_min_distance:
                    max_min_distance = min_dist
                    best_position = (new_lat, new_lon)
            
            # En iyi pozisyonu kullan
            if best_position:
                new_lat, new_lon = best_position
                existing_positions.append((new_lat, new_lon))
                
                # Pozisyonu listeye ekle
                positions.append({
                    'lat': new_lat,
                    'lon': new_lon,
                    'alt': center_alt  # Tüm dronlar aynı yükseklikte
                })
            else:
                # Eğer uygun pozisyon bulunamazsa, basit bir şekilde yeni bir pozisyon oluştur
                angle = random.uniform(0, 2 * math.pi)
                distance = spacing_meters * (i + 1) / 2.0  # Her drone için artan mesafe
                
                north_offset = distance * math.cos(angle)
                east_offset = distance * math.sin(angle)
                
                new_lat, new_lon = FormationFactory.calculate_gps_offset(
                    center_lat, center_lon, north_offset, east_offset)
                
                positions.append({
                    'lat': new_lat,
                    'lon': new_lon,
                    'alt': center_alt
                })
                
                existing_positions.append((new_lat, new_lon))
        
        return positions
        
    @staticmethod
    def assign_drones_to_targets(drones: List[Dict[str, Any]], 
                                targets: List[Dict[str, float]]) -> List[Tuple[int, Dict[str, float]]]:
        """
        Munkres (Hungarian) algoritmasını kullanarak drone'ları hedef pozisyonlara en verimli şekilde atar.
        
        Args:
            drones: Mevcut drone'ların listesi, her biri en azından {'id', 'lat', 'lon'} içermeli
            targets: Hedef pozisyonlar listesi, her biri {'lat', 'lon', 'alt'} içermeli
            
        Returns:
            List[Tuple[int, Dict[str, float]]]: (drone_id, hedef_pozisyon) eşleştirmeleri listesi
        """
        if not drones or not targets:
            return []
        
        # Maliyet matrisini oluştur (her drone-hedef çifti için mesafe)
        cost_matrix = np.zeros((len(drones), len(targets)))
        
        for i, drone in enumerate(drones):
            for j, target in enumerate(targets):
                cost_matrix[i, j] = FormationFactory.calculate_distance_meters(
                    drone['lat'], drone['lon'], target['lat'], target['lon'])
        
        # Munkres algoritması ile minimum toplam mesafeyi sağlayan eşleştirme bul
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        
        # Eşleştirmeleri döndür
        assignments = []
        for row, col in zip(row_indices, col_indices):
            assignments.append((drones[row]['id'], targets[col]))
        
        return assignments
