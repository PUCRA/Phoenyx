from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta al archivo de configuración YAML
    guiado_config = os.path.join(
        get_package_share_directory('guiado'),
        'config',
        'efk.yaml'
    )

    return LaunchDescription([
        Node(
            package='guiado',
            executable='localizacion_aruco',
            name='localizacion_aruco',
            output='screen',
            emulate_tty=True,
            parameters=[guiado_config],  # Carga el archivo YAML directamente
        ),
    ])