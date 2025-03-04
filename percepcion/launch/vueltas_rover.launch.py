from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    osr_bringup_dir = get_package_share_directory('osr_bringup')
    osr_mod_launch = os.path.join(osr_bringup_dir, 'launch', 'osr_mod_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(osr_mod_launch)
        ),
        Node(
            package='phoenyx_nodes',
            executable='imu_pub',
            name='imu_pub',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='percepcion',
            executable='dar_vueltas',
            name='dar_vueltas',
            output='screen',
            emulate_tty=True,
        ),
    ])