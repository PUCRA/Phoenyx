#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    lidar_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'X4-Pro.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = Node(package='ydlidar_ros2_driver',
                        executable='ydlidar_ros2_driver_node',
                        name='ydlidar_ros2_driver_node',
                        output='screen',
                        emulate_tty=True,
                        parameters=[parameter_file],
                        namespace='/',
                        # qos_profile=lidar_qos_profile,
                        )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    # arguments=['0', '0', '0.3','0', '0', '0', '1','base_link','laser_frame'],
                    arguments=['0', '0', '0.3', '0', '0', '-0.7071', '0.7071','base_link','laser_frame'],
                    respawn='True',
                    )
    # rviz2_node = Node(package='rviz2',
                    # executable='rviz2',
                    # name='rviz2',
                    # arguments=['-d', rviz_config_file],
                    # )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        # rviz2_node,
    ])
