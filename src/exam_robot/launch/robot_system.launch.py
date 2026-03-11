#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Упрощенный launch-файл для запуска системы робота exam_robot.
    """
    
    # Путь к URDF файлу
    pkg_share = get_package_share_directory('exam_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'exam_robot.urdf')
    
    # Читаем URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Список узлов для запуска
    nodes = [
        # Узлы из пакета exam_robot
        Node(package='exam_robot', executable='battery_node.py', name='battery_node', output='screen'),
        Node(package='exam_robot', executable='distance_sensor.py', name='distance_sensor', output='screen'),
        Node(package='exam_robot', executable='status_display.py', name='status_display', output='screen'),
        Node(package='exam_robot', executable='robot_controller.py', name='robot_controller', output='screen'),
        
        # Стандартные узлы ROS 2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ]
    
    return LaunchDescription(nodes)