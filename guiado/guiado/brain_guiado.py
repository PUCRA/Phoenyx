import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import subprocess
import time
import math
import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

class FSM(Node):
    def __init__(self):
        super().__init__('brain_guiado')

        #creamos publicadores en aruco_scan, se encarga de dar un trigger para escanear arucos
        self.publisher_ = self.create_publisher(Bool, '/aruco_scan', 1)
        self.publisher_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        #creamos subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback,1)
        self.create_subscription(Twist, '/aruco_pos', self.aruco_pos_callback, 1)
        self.waypoints = self.load_waypoints_yaml()
        
        self.state = 0  # Estado inicial 
        self.timer = self.create_timer(0.1, self.FSM)  # 0.1 segundos
        self.odometry_recived = False 
        self.aruco_pos_state = False
        
        self.waypoint_index = 0
        self.goal_reached = False
        self.arrival_time = None
        self.position = None  # Se actualizará con /odom

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.lidar_launched = False
        
        self.goal_accepted = False
        self.goal_sent = False
        self.tf_buffer = tf2_ros.Buffer()                
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.first = True
        self.distance = None



    def FSM(self):
        #S0: cuando recibe odometria ejecuta callback que activa un flag indicando que puede empezar la FSM
        if self.state == 0:
            if self.first:
                self.get_logger().info('Estado 0: controlando...')
                self.first = False
            
            if self.odometry_recived:
                time.sleep(5)
                self.launch_planner()
                time.sleep(20)  
                self.state = 1  # Cambia al siguiente estado 1
                self.first = True

        
        #S1: resetea la odometria dando un trigger en el topic aruco_scan 
        elif self.state == 1:
            if self.first:
                self.get_logger().info('Estado 1: publicando True en /aruco_scan y reseteando odometria')
                self.first = False
            if not hasattr(self, 'published_once'):
                self.published_once = False

            if not self.published_once:
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                time.sleep(0.5)
                msg.data = False
                self.publisher_.publish(msg)

                self.published_once = True  # Marca que ya se ha publicado

            elif self.aruco_pos_state:
                self.state = 2   # Ir al estado final
                self.first = True


        #S2: Resetea el mapa con la nueva odometria
        elif self.state == 2:
            if self.first:
                self.get_logger().info('Estado 2: Lanzando el Lidar')
                self.first = False
            if not self.lidar_launched:
                self.launch_Lidar()
                time.sleep(10)            
                self.state = 3  # Ir al estado final
                self.first = True


        #S3: Navegar a travès de los waypoints
        elif self.state == 3:
            if self.first:
                self.get_logger().info("Estado 3: enviando waypoints")
                self.first = False
            
            total_wp = len(self.waypoints)


            if self.waypoint_index < total_wp:
                wp = self.waypoints[self.waypoint_index]
                
                # Si no hemos enviado un goal válido todavía, lo intentamos
                if not self.goal_sent:
                    self.send_goal(wp['x'], wp['y'])
                    self.get_logger().info(f"Waypoint {self.waypoint_index + 1} enviado.")
                    # Marcamos que hemos intentado el envío, pero no que esté aceptado
                    self.goal_sent = True
                    self.goal_reached = False
                
                # Si ya está aceptado y luego completado, pasamos al siguiente
                if self.goal_reached:
                    self.get_logger().info(f"Waypoint {self.waypoint_index + 1} completado.")
                    self.waypoint_index += 1
                    self.goal_sent = False
                    self.goal_accepted = False
                    self.goal_reached = False
                    self.arrival_time = None
                    # time.sleep(5)
                
            else:
                self.get_logger().info("Todos los waypoints alcanzados.")
                self.state = 4
        
        #S5: Estado final de reposo 
        elif self.state == 5:
            self.get_logger().info('Estado 5: estado final alcanzado. Nada más que hacer.')
            self.timer.cancel()  # Detiene la máquina de estados

            
    def odom_callback(self, msg):    
        self.odometry_recived = True    # Se ha recibido un msg por /odom
    
    def aruco_pos_callback(self, msg):
        self.aruco_pos_state = True     # Se ha recibido un mgs por /aruco_pos
        # Extrae y almacena los datos
        self.initial_x = msg.linear.x
        self.initial_y = msg.linear.y
        self.initial_yaw = msg.angular.z  # Suponiendo que angular.z es el yaw
        self.publish_pose(0.0, 0.0, 0.0)
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
            self.goal_sent = False
            return
        
        elif goal_handle.accepted:                                              # Se ha cmabiado esto
            self.get_logger().info('Goal aceptado, esperando resultado...')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

 
    def get_result_callback(self, future):

        self.get_logger().warn("📋SE HA EJECUTADO CALLBACK RESPONSE📋")

        # result = future.result().result
        status = future.result().status
        if status == 4 or self.distance < 0.26:
            # self.goal_sent = False
            self.goal_reached = True
            self.get_logger().info('Goal alcanzado correctamente.')
            # time.sleep(1)
        else:
            self.get_logger().warn(f'La navegación terminó con estado: {status}')
        self.arrival_time = self.get_clock().now()

    
    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: Distancia restante {feedback.distance_remaining:.2f}')
        self.distance = feedback.distance_remaining


        
    def launch_Lidar(self):
        subprocess.Popen(([
            'ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch_view.py',
        ]))
            
    def publish_pose(self, x, y, theta):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "odom"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0

            # Convert theta (yaw) to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, theta)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            # Covariance: usually set to low values for high confidence
            msg.pose.covariance = [0.0] * 36

            self.publisher_initialpose.publish(msg)
            self.get_logger().info(f"Published initial pose: x={x}, y={y}, theta={theta}")

    def launch_planner(self):
        subprocess.Popen(
            ['ros2', 'launch', 'guiado', 'planificador.launch.py']  # Falta añadir el yaml de guiado
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

