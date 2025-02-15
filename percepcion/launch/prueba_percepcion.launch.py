import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(file_path, namespace):
    """ Carga solo los parámetros de un namespace específico de un archivo YAML """
    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            return data.get(namespace, {}).get("ros__parameters", {})
    except Exception as e:
        print(f"Error cargando YAML: {e}")
        return {}

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

    orbbec_params = load_yaml(lectura_cam_params, "orbbec_camera_launch")
    ld.add_action(Node(
            package="orbbec_camera",
            executable="orbbec_camera_node",
            name="ob_camera_node",
            namespace='camera',
            parameters=[orbbec_params],
            output="screen",
        )
    )
    return ld
