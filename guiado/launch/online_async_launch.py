from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file = LaunchConfiguration('params_file')
    # map_start_pose = LaunchConfiguration('map_start_pose')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('guiado'), 'config', 'mapper_params_online_async_guiado.yaml'),
            description='Path to the parameter file'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[param_file,
                        {'use_sim_time': False},
                        """{'map_start_pose': map_start_pose}"""],
            # remappings=[('/scan', '/scan')],
            emulate_tty=True,
        ),
    ])
