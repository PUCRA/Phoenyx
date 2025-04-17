from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('control'),
        'config',
        'params.yaml'
    )
    bringup = os.path.join(
        get_package_share_directory('osr_bringup'),
        'launch',
        'osr_mod_launch.py'
    )
    gazebo = os.path.join(
        get_package_share_directory('osr_gazebo'),
        'launch',
        'circuito_paredes.launch.py'
    )
    slam_toolbox = os.path.join(
        get_package_share_directory('phoenyx_nodes'),
        'launch',
        'online_async_launch.py'
    )
    planificador = os.path.join(
        get_package_share_directory('planificador'),
        'launch',
        'planificador.launch.py'
    )


    with open(param_file, 'r') as f:
        params = yaml.safe_load(f)

    simulation = params.get('launch_config', {}).get('simulation', False)

    actions = []
    if simulation:
        # Aquí puedes agregar la lógica para lanzar el nodo de simulación
        actions.append(
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo)
        ))
    else:
        actions.append(
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup)
        ))
    

    return LaunchDescription([
        
        *actions,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planificador),
        ),
        # También puedes lanzar cosas propias aquí
        Node(
            package='control',
            executable='linea_media',
            name='linea_media',
            output='screen',
        ),
    ])
