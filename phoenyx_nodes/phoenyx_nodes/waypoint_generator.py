#!/usr/bin/env python3
import os
import yaml
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Cargar waypoints desde YAML
        self.waypoints = self.load_waypoints_yaml()
        self.get_logger().info(f"Waypoints cargados: {self.waypoints}")

        # Suscripción a /aruco_pos
        self.create_subscription(Twist, '/aruco_pos', self.aruco_callback, 10)

        # Estado de detección y navegación
        self.aruco_detected = False
        self.waypoint_reached = True  # Solo detectamos ArUco cuando un waypoint es alcanzado
        
        # Convertir waypoints a PoseStamped
        self.pose_waypoints = []
        for pos in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = pos['x']
            pose.pose.position.y = pos['y']
            pose.pose.position.z = 0.0
            qz = math.sin(pos['yaw'] / 2.0)
            qw = math.cos(pos['yaw'] / 2.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            self.pose_waypoints.append(pose)

        self.current_waypoint_index = 0

        if self.pose_waypoints:
            self.get_logger().info("Esperando detección de ArUco para iniciar el movimiento.")
        else:
            self.get_logger().warn("No se han encontrado waypoints en el archivo YAML.")

    def aruco_callback(self, msg):
        """Callback para la detección de ArUco. Solo activa cuando hemos alcanzado un waypoint o al inicio."""
        if not self.waypoint_reached:
            return  # Ignorar detecciones durante la navegación

        self.aruco_detected = True
        self.waypoint_reached = False  # Marcamos que iniciamos navegación
        
        if self.current_waypoint_index == 0:
            self.get_logger().info("¡Aruco detectado! Moviéndome al primer waypoint.")
        else:
            self.get_logger().info("¡Aruco detectado! Continuando con el siguiente waypoint.")
        
        self.send_next_waypoint()
    
    def load_waypoints_yaml(self):
        """Carga los waypoints desde un archivo YAML."""
        package_name = 'guiado'
        package_dir = get_package_share_directory(package_name)
        yaml_path = os.path.join(package_dir, 'config', 'waypoints.yaml')

        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar el YAML: {e}")
            return []

    def send_next_waypoint(self):
        """Envía el siguiente waypoint."""
        if self.current_waypoint_index >= len(self.pose_waypoints):
            self.get_logger().info("Todos los waypoints han sido alcanzados.")
            return

        waypoint = self.pose_waypoints[self.current_waypoint_index]
        waypoint.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Enviando waypoint {self.current_waypoint_index + 1}")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("El servidor de acción no está disponible.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback cuando se recibe la respuesta del servidor de acción."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("El objetivo fue rechazado.")
            return

        self.get_logger().info("Objetivo aceptado. Esperando resultado...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback cuando se recibe el resultado de la acción."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} alcanzado.")
            self.current_waypoint_index += 1
            self.waypoint_reached = True  # Permitimos la detección de ArUco de nuevo
        else:
            self.get_logger().warn(f"Fallo al alcanzar waypoint {self.current_waypoint_index + 1}")
        
    def feedback_callback(self, feedback_msg):
        """Callback de feedback mientras el robot se mueve."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback recibido: {feedback}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
