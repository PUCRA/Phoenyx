#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Cliente de acción para enviar objetivos a Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Definición de 6 waypoints (puedes ajustar las posiciones según tus necesidades)
        self.waypoints = []
        positions = [
            {'x': -3.0, 'y': 3.0, 'yaw': 0.0},
            {'x': 3.0, 'y': -3.0, 'yaw': 0.0},
            {'x': 3.0, 'y': 3.0, 'yaw': 0.0},
            {'x': -3.0, 'y': -3.0, 'yaw': 0.0},
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            
        ]
        for pos in positions:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            # La cabecera se actualizará antes de enviar el objetivo
            pose.pose.position.x = pos['x']
            pose.pose.position.y = pos['y']
            pose.pose.position.z = 0.0
            # Convertimos el yaw a una cuaternion (asumiendo roll=yaw=0)
            import math
            from geometry_msgs.msg import Quaternion
            qz = math.sin(pos['yaw'] / 2.0)
            qw = math.cos(pos['yaw'] / 2.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            self.waypoints.append(pose)

        self.current_waypoint_index = 0
        # Enviar el primer waypoint
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Todos los waypoints han sido alcanzados.")
            return

        # Actualizar la cabecera (timestamp) antes de enviar
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Enviando waypoint {self.current_waypoint_index + 1}")
        
        # Esperar a que el servidor de acción esté disponible
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("El servidor de acción no está disponible.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        # Enviar el objetivo a la acción
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("El objetivo fue rechazado.")
            return

        self.get_logger().info("Objetivo aceptado. Esperando resultado...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index +1} alcanzado.")
            self.current_waypoint_index += 1
            self.send_next_waypoint()

        # Enviar el siguiente waypoint
        else:
            self.get_logger().warn(f"Fallo al alcanzar waypoint {self.current_waypoint_index + 1}")
            self.send_next_waypoint()

    def feedback_callback(self, feedback_msg):
        # Opcional: Procesar retroalimentación durante la navegación
        feedback = feedback_msg.feedback
        # Por ejemplo, se puede mostrar información de progreso
        self.get_logger().info(f"Feedback: {feedback}")

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
