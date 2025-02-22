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

    # ld.add_action(
    #     Node(
    #         package='percepcion',
    #         executable='lectura_camara',
    #         name='lectura_camara',
    #         parameters=[lectura_cam_params]
    #     )
    # )

    orbbec_params = load_yaml(lectura_cam_params, "orbbec_camera_launch")
    ld.add_action(
        Node(
            package="orbbec_camera",
            executable="orbbec_camera_node",
            name="ob_camera_node",
            namespace='camera',
            parameters=[orbbec_params],
            output="screen",
        )
    )
    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'roboclaw_params_mod.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params_mod.yaml'
    )

    # ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package='osr_control',
            executable='roboclaw_wrapper',
            name='roboclaw_wrapper',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[roboclaw_params]
        )
    )

    ld.add_action(
        Node(
            package='osr_control',
            executable='servo_control',
            name='servo_wrapper',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[{'centered_pulse_widths': [157, 147, 152, 147]}]  # pulse width where the corner motors are in their default position, see rover_bringup.md.
        )
    )

    ld.add_action(
        Node(
            package='osr_control',
            executable='rover',
            name='rover',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[osr_params,
                        {'enable_odometry': True}]
        )
    )

    ld.add_action(
        Node(
            package='osr_control',
            executable='ina260',
            name='ina260_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"publish_rate": 1.0},
                {"sensor_address": "0x44"},
            ]        
        )
    )

    # ld.add_action(
    #     Node(
    #         package='osr_control',
    #         executable='killer_node',
    #         name='killer_node',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[]
    #     )
    # )

    ld.add_action(
        Node(
            package='phoenyx_nodes',
            executable='imu_pub',
            name='imu_pub',
            output='screen',
            emulate_tty=True,
            parameters=[]
        )
    )

    ld.add_action(
        Node(
            package='percepcion',
            executable='dar_vueltas',
            name='dar_vueltas',
            output='screen',
            emulate_tty=True,
            parameters=[]
        )
    )
    ld.add_action(
        Node(
            package='percepcion',
            executable='brain_percepcion',
            name='brain_percepcion',
            output='screen',
            emulate_tty=True,
            # parameters=[]
            parameters=[lectura_cam_params]
        )
    )
    return ld
