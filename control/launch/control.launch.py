from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.actions import TimerAction

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('control'),
        'config',
        'params.yaml'
    )

    lidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params',
        'X4-Pro.yaml'
    )
    lidar = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch_view.py'
    )
    bringup = os.path.join(
        get_package_share_directory('osr_bringup'),
        'launch',
        'osr_mod_launch.py'
    )


    slam_params = os.path.join(
        get_package_share_directory('phoenyx_nodes'), 
        'conf', 
        'mapper_params_online_async.yaml')
    slam_toolbox = os.path.join(
        get_package_share_directory('phoenyx_nodes'),
        'launch',
        'online_async_launch.py'
    )

    # planificador_params = os.path.join(
    #     get_package_share_directory('planificador'),
    #     'launch',
    #     'planifi'
    # )
    # planificador = os.path.join(
    #     get_package_share_directory('planificador'),
    #     'launch',
    #     'planificador.launch.py'
    # )

    nodes_delayed = TimerAction(
        period= 40.0,  # segundos
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar),
                launch_arguments={'params_file': lidar_params}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox),
                launch_arguments={'params_file': slam_params}.items()
            ),
        ]
    )
    nodes_delayed_less = TimerAction(
        period=5.0,  # segundos
        actions=[
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(planificador),
            # ),
            Node(
                package='control',
                executable='linea_media',
                name='linea_media',
                output='screen',
            ),
        ]
    )


    with open(param_file, 'r') as f:
        params = yaml.safe_load(f)

    simulation = params.get('launch_config', {}).get('simulation', False)

    actions = []
    if simulation:
        # Aquí puedes agregar la lógica para lanzar el nodo de simulación
        gazebo = os.path.join(
        get_package_share_directory('osr_gazebo'),
        'launch',
        'circuito_paredes.launch.py'
        )
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
        nodes_delayed,
        nodes_delayed_less
    ])
