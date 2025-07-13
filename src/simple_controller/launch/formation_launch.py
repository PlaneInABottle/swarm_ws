#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Formation Manager Launch File

Bu launch dosyası mevcut sürü sisteminize formasyon yöneticisini ekler.

Kullanım:
    ros2 launch simple_controller formation_launch.py formation_type:=v drone_spacing:=5.0 enable_formation:=true

Parametreler:
    formation_type: Formasyon tipi (v, line, arrowhead)
    drone_spacing: Dronlar arası mesafe (metre)
    enable_formation: Formasyon kontrolü aktif/pasif (true/false)
    navigation_acceptance_radius: Navigasyon hedef kabul yarıçapı (metre)
    navigation_speed: Navigasyon hızı (m/s)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Launch argümanlarını tanımla
    formation_type_arg = DeclareLaunchArgument(
        'formation_type',
        default_value=TextSubstitution(text='v'),
        description='Formasyon tipi (v, line, arrowhead)'
    )
    
    drone_spacing_arg = DeclareLaunchArgument(
        'drone_spacing',
        default_value=TextSubstitution(text='5.0'),
        description='Dronlar arası mesafe (metre)'
    )
    
    enable_formation_arg = DeclareLaunchArgument(
        'enable_formation',
        default_value=TextSubstitution(text='true'),
        description='Formasyon kontrolünü aktif/pasif yap (true/false)'
    )
    
    navigation_acceptance_radius_arg = DeclareLaunchArgument(
        'navigation_acceptance_radius',
        default_value=TextSubstitution(text='1.0'),
        description='Navigasyon hedef kabul yarıçapı (metre)'
    )
    
    navigation_speed_arg = DeclareLaunchArgument(
        'navigation_speed',
        default_value=TextSubstitution(text='1.5'),
        description='Navigasyon hızı (m/s)'
    )

    # Formation Manager node'unu başlat
    formation_manager_node = Node(
        package='simple_controller',
        executable='formation_manager_node',
        name='formation_manager',
        output='screen',
        parameters=[{
            'formation_type': LaunchConfiguration('formation_type'),
            'drone_spacing': LaunchConfiguration('drone_spacing'),
            'enable_formation': LaunchConfiguration('enable_formation'),
            'navigation_acceptance_radius': LaunchConfiguration('navigation_acceptance_radius'),
            'navigation_speed': LaunchConfiguration('navigation_speed'),
        }]
    )

    # ExecuteProcess ile lifecycle_transition komutu otomatik olarak çalıştır
    # Bu, SwarmStatePublisherNode'un otomatik olarak aktifleştirilmesini sağlar
    activate_swarm_state_publisher_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/swarm_state_publisher_node', 'configure'],
        shell=True,
        output='screen'
    )
    
    activate_swarm_state_publisher_cmd2 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/swarm_state_publisher_node', 'activate'],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        # Launch argümanları
        formation_type_arg,
        drone_spacing_arg,
        enable_formation_arg,
        navigation_acceptance_radius_arg,
        navigation_speed_arg,
        
        # Node ve execute process aksiyonları
        formation_manager_node,
        activate_swarm_state_publisher_cmd,
        activate_swarm_state_publisher_cmd2
    ])
