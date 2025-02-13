#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import os


def generate_launch_description():
    # Usar la política de QoS RELIABLE
    lidar_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    # Directorio compartido del paquete
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    
    # Declarar el archivo de parámetros
    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'X4-Pro.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    # Nodo para el LiDAR
    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                qos_profile=lidar_qos_profile)  # Aplicar RELIABLE al LiDAR

    # Nodo para publicar la transformación estática
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
                    )

    # Incluir el nodo slam_toolbox y aplicar la misma política de QoS (RELIABLE)
    slam_node = Node(package='slam_toolbox',
                     executable='async_slam_toolbox_node',
                     name='slam_toolbox_node',
                     output='screen',
                     parameters=[{'slam_params_file': '/path/to/your/slam_params.yaml'}],
                     remappings=[('/scan', '/scan')],  # Asegúrate de que el topic coincida
                     qos_profile=lidar_qos_profile)  # Aplicar RELIABLE a slam_toolbox

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        slam_node,  # Incluir el nodo SLAM
    ])


tf2_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_pub_laser',
    arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
)
