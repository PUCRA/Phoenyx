import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from slam_toolbox.srv import DeserializePoseGraph
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from math import sin, cos
from geometry_msgs.msg import Pose2D
import subprocess
import time



class FSM(Node):
    def __init__(self):
        super().__init__('brain_guiado')
        #creamos publicadores en aruco_scan, se encarga de dar un trigger para escanear arucos
        self.publisher_ = self.create_publisher(Bool, '/aruco_scan', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        #creamos subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback,10)
        self.create_subscription(Twist, '/aruco_pos', self.aruco_pos_callback, 10)
        self.client = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self.waypoints = self.load_waypoints_yaml()

        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Esperando al servicio /slam_toolbox/deserialize_map...')
        
        self.state = 0  # Estado inicial 
        self.timer = self.create_timer(0.1, self.FSM)  # 0.1 segundos
        self.odometry_recived = False 
        self.aruco_pos_state = False
        
        self.waypoint_index = 0
        self.goal_reached = False
        self.arrival_time = None
        self.position = None  # Se actualizará con /odom

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info('Esperando al servidor de la acción navigate_to_pose...')



    def FSM(self):
        #S0: cuando recibe odometria ejecuta callback que activa un flag indicando que puede empezar la FSM
        if self.state == 0:
            self.get_logger().info('Estado 0: controlando...')
            # Si recibe de /odom
            if self.odometry_recived:
                self.state = 1  # Cambia al siguiente estado, aqui va un 1

        
        #S1: resetea la odometria dando un trigger en el topic aruco_scan 
        elif self.state == 1:
            self.get_logger().info('Estado 1: publicando True en /aruco_scan y reseteando odometria')
            
            if not hasattr(self, 'published_once'):
                self.published_once = False

            if not self.published_once:
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                time.sleep(0.01)
                msg.data = False
                self.publisher_.publish(msg)

                self.published_once = True  # Marca que ya se ha publicado

            elif self.aruco_pos_state:
                self.state = 2  # Ir al estado final


        #S2: Resetea el mapa con la nueva odometria
        elif self.state == 2:
            self.get_logger().info('Estado 2: Reset del mapa')
            
            self.launch_SLAM(self.initial_x, self.initial_y, self.initial_yaw)

            time.sleep(5)
            
            self.state = 3  # Ir al estado final


        #S3: Navegar a travès de los waypoints
        elif self.state == 3:
            total_wp = len(self.waypoints)

            if self.waypoint_index < total_wp:
                # Si ya se alcanzó el goal, esperar 5 segundos antes de avanzar
                if self.goal_reached:
                    if self.arrival_time and (self.get_clock().now() - self.arrival_time).nanoseconds > 5e9:
                        self.get_logger().info(f"Waypoint {self.waypoint_index + 1} completado.")
                        self.waypoint_index += 1
                        self.goal_reached = False  # Reinicia para el siguiente waypoint
                        self.arrival_time = None
                else:
                    # Si todavía no se ha enviado el goal, envíalo.
                    # Asegurarse de que no se envíe repetidamente:
                    if not hasattr(self, 'goal_sent') or not self.goal_sent:
                        wp = self.waypoints[self.waypoint_index]
                        # Suponiendo que cada waypoint es [x, y]
                        self.send_goal(wp['x'], wp['y'])
                        self.goal_sent = True
            else:
                self.get_logger().info("Todos los waypoints alcanzados.")
                self.state = 4

            
        #S4: Estado final de reposo 
        elif self.state == 4:
            self.get_logger().info('Estado 4: estado final alcanzado. Nada más que hacer.')
            self.timer.cancel()  # Detiene la máquina de estados

            
    def odom_callback(self, msg):    
        self.odometry_recived = True    # Se ha recibido un msg por /odom
    
    def aruco_pos_callback(self, msg):
        self.aruco_pos_state = True     # Se ha recibido un mgs por /aruco_pos
        # Extrae y almacena los datos
        self.initial_x = msg.linear.x
        self.initial_y = msg.linear.y
        self.initial_yaw = msg.angular.z  # Suponiendo que angular.z es el yaw

        self.get_logger().info(
            f"Aruco pos: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}"
        )

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

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = 0.0

        pose_msg = PoseStamped()
        pose_msg.header = goal_msg.pose.header
        pose_msg.pose = goal_msg.pose.pose
        self.goal_pose_pub.publish(pose_msg)

        self.get_logger().info(f"Enviando goal de navegación: x={x}, y={y}")
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rechazado.')
            # Puedes definir alguna lógica de reintento o marcar el goal como alcanzado para continuar.
            self.goal_reached = True
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal alcanzado correctamente.')
        else:
            self.get_logger().warn(f'La navegación terminó con estado: {status}')
        self.goal_reached = True
        self.arrival_time = self.get_clock().now()
        self.goal_sent = False  # Permite enviar el siguiente goal


    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Distancia restante {feedback.distance_remaining:.2f}')

    def callback_response(self, future):
        try:
            result = future.result()
            if result.result:
                self.get_logger().info("Mapa cargado correctamente.")
            else:
                self.get_logger().error("La carga del mapa falló.")
        except Exception as e:
            self.get_logger().error(f"Error al cargar el mapa: {e}")
    
    def launch_SLAM(self, x, y, yaw):
        pose_param = f"[{x-1.0}, {y-1.0}, {yaw}]"
        subprocess.Popen(([
            'ros2', 'launch', 'guiado', 'online_async_launch.py',
            'use_sim_time:=false',
            'map_start_pose:='+ pose_param
        ]))
        self.get_logger().info(pose_param)


def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




