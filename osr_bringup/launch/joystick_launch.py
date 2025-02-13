from launch import LaunchDescription
from launch_ros.actions import Node

import os 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'joystick.yaml'
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[joy_params],
    )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[joy_params]
    )


    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
    ])