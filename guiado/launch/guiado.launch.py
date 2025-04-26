import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():

    launch_SLAM = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('guiado').find('guiado'),
                'launch',
                'online_async_launch.py'
            )
        )
    )

    launch_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('osr_bringup').find('osr_bringup'),
                'launch',
                'osr_mod_launch.py'
            )
        )
    )
        
    # Nodo de localización de ArUco
    localizacion_node = Node(
        package='guiado',
        executable='localizacion_aruco',
        name='localizacion_aruco',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    #Incluir otro archivo de launch
    # launch_planificador = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(
            # os.path.join(
                # FindPackageShare('planificador').find('planificador'),
                # 'launch',
                # 'planificador.launch.py'  # <- Cambia esto si se llama diferente
            # )
        # )
    # )

    brain_node = Node(
        package='guiado',
        executable='brain_guiado',
        name='brain_guiado',
        output='screen',
        parameters=[{'use_sim_time':False}]
    )

    
          
    return LaunchDescription([
    launch_SLAM,

    TimerAction(period=4.0, actions=[
        launch_bringup
    ]),

    TimerAction(period=4.0, actions=[
        localizacion_node
    ]),

    # TimerAction(period=30.0, actions=[
        # launch_planificador
    # ]),

    TimerAction(period=30.0, actions=[
        brain_node
    ]),

])
