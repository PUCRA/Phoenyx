from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
import launch

def generate_launch_description():
    # Ruta al archivo de configuración YAML
    guiado_config = os.path.join(
        get_package_share_directory('guiado'),
        'config',
        'efk.yaml'
    )
    

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[guiado_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='guiado',
            executable='localizacion_aruco',
            name='localizacion_aruco',
            output='screen',
            emulate_tty=True,
            parameters=[guiado_config],  # Carga el archivo YAML directamente
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'
        ),
        # robot_state_publisher_node,  # Todo esto que esta comentado lo indica en el tutorial, perono esta definido y no se como definirlo
        # spawn_entity,
        # robot_localization_node,
        # rviz_node
    ])