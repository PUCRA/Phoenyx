from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener ruta al launch file de Gazebo
    osr_gazebo_dir = get_package_share_directory('osr_gazebo')
    gazebo_launch_path = os.path.join(osr_gazebo_dir, 'launch', 'circuito_arucos.launch.py')
    osr_joy_dir = get_package_share_directory('osr_bringup')
    joy_launch=os.path.join(osr_joy_dir, 'launch','joystic_launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch)
    )

    # Nodo 1: localizacion_aruco.py desde paquete guiado
    localizacion_aruco_node = Node(
        package='guiado',
        executable='localizacion_aruco',  # o solo 'localizacion_aruco' si lo registraste así en setup.py
        name='localizacion_aruco',
        output='screen'
    )

    # Nodo 2: tests_odom_aruco desde paquete phoenyx_nodes
    tests_odom_node = Node(
        package='phoenyx_nodes',
        executable='tests_odom_aruco',
        name='tests_odom_aruco',
        output='screen'
    )

    # Lanza Gazebo primero, y después de 5s los nodos Python
    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=5.0, actions=[
            localizacion_aruco_node,
            tests_odom_node
        ])
    ])
