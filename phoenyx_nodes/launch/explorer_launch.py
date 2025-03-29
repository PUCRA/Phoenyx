from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='phoenyx_nodes',
            executable='explorer_node',
            name='nav2_explorer_node',
            output='screen',
        )
    ])
