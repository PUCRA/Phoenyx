import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Ruta al archivo de mundo directamente especificada
    world_file = './src/osr_gazebo/worlds/prueba_final.world'

    # Verificar que la ruta al archivo del mundo sea correcta
    print("Ruta al archivo del mundo:", world_file)
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"El archivo de mundo no se encuentra en: {world_file}")

    # Lanzamiento de Gazebo con el mundo personalizado
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'use_sim_time': 'true'}.items(),  # use_sim_time agregado aquí
    )

    # Ruta al archivo URDF/Xacro
    osr_gazebo_path = get_package_share_directory('osr_gazebo')
    xacro_file = os.path.join(osr_gazebo_path, 'urdf', 'osr_mod.urdf.xacro')
    
    # Verificar si el archivo Xacro existe
    print("Ruta al archivo Xacro:", xacro_file)
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"El archivo Xacro no se encuentra en: {xacro_file}")

    # Procesar el archivo Xacro
    try:
        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)
    except Exception as e:
        raise RuntimeError(f"Error al procesar el archivo Xacro: {e}")

    # Parámetros para el robot_description
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}  # use_sim_time agregado aquí

    # Nodo para publicar el estado del robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Nodo para controlar el robot
    controller_spawn = Node(
        package='osr_gazebo',
        executable='osr_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]  # use_sim_time agregado aquí también
    )
    
    # Nodo para spawn de la entidad en Gazebo con coordenadas
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rover',
            '-x', '1.0', '-y', '1.0', '-z', '0',  # Coordenadas de posición
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]  # use_sim_time agregado aquí también
    )

    # Controladores
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_controller'],
        output='screen'
    )

    servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )

    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        controller_spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    rover_wheel_controller,
                    servo_controller,
                ],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
