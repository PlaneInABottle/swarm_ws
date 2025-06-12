#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import random
from typing import List, Dict, Any, Tuple

from .formation_factory import FormationFactory

class RandomFormationGenerator:
    """
    Drone sürüleri için rastgele formasyon stratejileri oluşturur.
    Dronlar arası mesafe parametresini kullanarak sürünün merkez noktası etrafında
    rastgele pozisyonlar oluşturur.
    
    swarm_centroid merkez noktası olarak kullanılır ve tüm hesaplamalar bu merkez etrafında yapılır.
    """
    
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
