import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lectura_cam_params = os.path.join(
            get_package_share_directory('percepcion'),
            'config',
            'lectura_cam.yaml'
        )
    
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package='percepcion',
            executable='lectura_camara',
            name='lectura_camara',
            parameters=[lectura_cam_params]
        )
    )

    return ld
