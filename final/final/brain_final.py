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
# import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import rclpy.duration
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Joy, PointCloud2
from std_msgs.msg import Header
from bond.msg import Status

class FSM_final(Node):
    def __init__(self):
        super().__init__('brain_final')
        #### ================= PERCEPCION ================= ####





        #### ================= GUIADO ================= ####

        #creamos publicadores en aruco_scan, se encarga de dar un trigger para escanear arucos
        self.publisher_ = self.create_publisher(Bool, '/aruco_scan', 1)
        self.publisher_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        #creamos subscripciones
        # self.create_subscription(Odometry, '/odom', self.odom_callback,1)
        self.create_subscription(Twist, '/aruco_pos', self.aruco_pos_callback, 1)
        self.subscription = self.create_subscription(
            Status,
            '/bond',
            self.bond_callback,
            10
        )
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
        self.nav2_ready = False


        #### ================= CONTROL ================= ####

        self.goal_distance = 2.0  # distancia hacia adelante
        self.goal_threshold = 1.0  # metros para anticipar siguiente goal
        self.timeout = 2.0
        self.goal_active = False
        self.prev_time = 0
        self.last_goal_pose = None
        # self.last_goal_angle = None
        self.lidar_msg = None
        # self.clock = self.get_clock()
        self.frame_id = 'base_link'
        self.map_frame = 'map'

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_points = self.create_publisher(PointStamped, '/points', 10)
        self.joystick = self.create_subscription(
            Joy,
            '/joy',
            self.callback_mando,
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # tamaño del buffer
        )
        self.x = 0
        self.y = 0
        self.orientation_q = None

        self.tf_buffer = tf2_ros.Buffer()                
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.FSM = self.create_timer(0.1, self.brain)
        self.start_node = False
        self.last_angle = 0.0
        # self.get_logger().info("⏩ Navegación continua con LiDAR iniciada")



    def FSM(self):
        #### ================= PERCEPCION ================= ####
        
        


        #### ================= LOCALIZACION ================= ####
        
        #S0: cuando recibe odometria ejecuta callback que activa un flag indicando que puede empezar la FSM
        
        if self.state == 0:
            if self.first:
                self.get_logger().info('Estado 0: controlando...')
                self.first = False
            
            if self.odometry_recived:
                time.sleep(1)
                self.state = 3  # Cambia al siguiente estado 1
                self.first = True
        
        elif self.state == 1:
            if self.first:
                self.get_logger().info('Estado 1: publicando True en /aruco_scan y reseteando odometria')
                self.first = False
            if not hasattr(self, 'published_once'):
                self.published_once = False

            if not self.published_once:
                time.sleep(5)
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                time.sleep(0.5)
                msg.data = False
                self.publisher_.publish(msg)
                self.get_logger().info('Estado 1: Publicado!')
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
                time.sleep(8)
                self.launch_planner()
                self.state = 3  # Ir al estado final
                self.first = True


        #### ================= WAYPOINTS ================= ####

        #S3: Esperar a nav2
        elif self.state == 3:
            if self.first:
                self.get_logger().info('Estado 3: Esperando a que NAV2 esté listo')
                self.first = False
            if self.nav2_ready:
                self.first = True
                time.sleep(0.5)
                self.state = 4

        #S4: Navegar a travès de los waypoints
        elif self.state == 4:
            if self.first:
                self.get_logger().info("Estado 4: enviando waypoints")
                self.first = False
            
            if not self.goal_reached:
                wp = self.waypoints[0]
                
                # Si no hemos enviado un goal válido todavía, lo intentamos
                if not self.goal_sent:
                    self.send_goal(wp['x'], wp['y'])
                    self.get_logger().info("Waypoint pasillo enviado.")
                    # Marcamos que hemos intentado el envío, pero no que esté aceptado
                    self.goal_sent = True
                    self.goal_reached = False
                
            else:
                self.get_logger().info("Todos los waypoints alcanzados.")
                self.state = 5
                self.first = True


        #### ================= CONTROL ================= ####

        elif self.state == 5:
            if self.first:
                self.get_logger().info("Estado 5: Navegando el pasillo")
                self.first = False

            debug = True
            if debug:
                if self.lidar_msg != None and self.start_node:
                    if self.goal_active:
                        distance = self.check_progress()
                        self.get_logger().info(f"Distancia al goal: {distance:.2f} m")
                        if distance != -1 and distance < self.goal_threshold:
                            self.get_logger().info(f"📍 Cerca del goal ({distance:.2f} m)")
                            self.goal_active = False
                        if time.time() - self.prev_time > self.timeout:
                            self.goal_active = False
            else:
                self.state = 6


        #### ================= FINAL ================= ####
        
        #S6: Estado final de reposo 
        elif self.state == 6:
            self.get_logger().info('Estado 6: estado final alcanzado. Nada más que hacer.')
            self.timer.cancel()  # Detiene la máquina de estados



    #### ================= FUNCIONES PERCEPCION ================= ####
    
    
    
    
    
    #### ================= FUNCIONES GUIADO ================= ####     
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
       package_name = 'final'
       package_dir = get_package_share_directory(package_name)
       yaml_path = os.path.join(package_dir, 'config', 'waypoints_final.yaml')
       try:
           with open(yaml_path ,'r') as file:
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
        self._send_goal_future.add_done_callback(self.goal_response_callback_1)

    def goal_response_callback_1(self, future):
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
        if status == 4 or self.distance < 0.5:
            # self.goal_sent = False
            self.get_logger().info('Goal alcanzado correctamente.')
            time.sleep(5.1)
            self.goal_reached = True
            # time.sleep(1)
        elif status == 6:
            self.get_logger().info('Goal abortado.')
            self.goal_sent = False
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
            qx, qy, qz, qw = self.euler_to_quaternion(0.0 ,0.0, theta)
            msg.pose.pose.orientation.x = qx #q[0]
            msg.pose.pose.orientation.y = qy #q[1]
            msg.pose.pose.orientation.z = qz #q[2]
            msg.pose.pose.orientation.w = qw #q[3]

            # Covariance: usually set to low values for high confidence
            msg.pose.covariance = [0.0] * 36

            self.publisher_initialpose.publish(msg)
            self.get_logger().info(f"Published initial pose: x={x}, y={y}, theta={theta}")

    def launch_planner(self):
        subprocess.Popen(
            ['ros2', 'launch', 'planificador', 'planificador.launch.py'] # Falta añadir el yaml de guiado
        )
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        # semicírculos
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
    
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
    
        return qx, qy, qz, qw

    def timer_callback():
        return
    

    def bond_callback(self, msg):
        if not self.nav2_ready:
            self.nav2_ready = True
            self.get_logger().info("¡bt_navigator está activo y publicando estado!")



    #### ================= FUNCIONES CONTROL ================= ####


    def callback_mando(self, msg):
        if (not self.start_node) and msg.buttons[0]:
            self.get_logger().info("Iniciando nodo")
            self.start_node = msg.buttons[0] # Boton A
                
    # Actualiza el mensaje del lidar
    def lidar_callback(self, msg):
        if (not self.goal_active) and self.start_node:
            self.lidar_msg = msg
            self.get_logger().info("Generando siguiente goal...")
            x_forward, y_lateral, yaw = self.generate_goal_from_lidar(self.lidar_msg)
            if x_forward is None:
                return
            goal, error = self.create_and_send_goal(x_forward, y_lateral, yaw)
            self.goal_active = not error
            # if self.goal_active:
            #     time.sleep(5.0)
            self.prev_time = time.time()
            if error:
                self.get_logger().warning("Error al generar el goal")
            else:
                self.last_goal_pose = goal
            

    # Checkea cuanta distancia queda para llegar al goal
    def check_progress(self):
        if not self.goal_active or self.last_goal_pose is None:
            return -1
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            goal_x = self.last_goal_pose.pose.position.x
            goal_y = self.last_goal_pose.pose.position.y

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            return distance
        
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"[TF Error] al verificar progreso: {e}")
            return -1


    def generate_goal_from_lidar(self, msg):
        # msg = self.rotate_laserscan(msg, np.radians(-90))
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Usamos un rango frontal de -80° a 80° (más sensible a giros)
        mask = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(80))
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        if len(valid_ranges) == 0:
            self.get_logger().warn("🚧 No hay datos válidos en -80° a 80°.")
            return

        # Suavizamos para quitar ruido
        smooth_ranges = np.convolve(valid_ranges, np.ones(3)/3, mode='same')
        new_ranges = []
        new_angles = []

        # Filtramos puntos cercanos
        for i in range(0, len(smooth_ranges)):
            # Filtramos puntos muy cercanos a la pared (por ejemplo, < 1.0 metros)
            if smooth_ranges[i] > 1.0:
                new_ranges.append(smooth_ranges[i])
                new_angles.append(valid_angles[i])

        if len(new_ranges) == 0:
            self.get_logger().warn("🚧 No hay puntos válidos después del filtrado.")
            return

        # Promediamos los puntos LIDAR en bloques de 10
        valid_ranges_2, valid_angles_2 = self.average_lidar_in_blocks(new_ranges, new_angles, block_size=10)

        # Calculamos las posiciones en x e y de los puntos promediados
        x_points = valid_ranges_2 * np.cos(valid_angles_2)
        y_points = valid_ranges_2 * np.sin(valid_angles_2)

        # Calculamos la media de las posiciones x e y
        goal_x = np.mean(x_points)
        goal_y = np.mean(y_points)

        mask = (np.isfinite(ranges)) & (np.radians(-5) <= angles) & (angles <= np.radians(5))
        front_distance = ranges[mask]
        front_distance = max(front_distance)
        if front_distance < 3.0:
            self.get_logger().warning(f"Pared detectada: {front_distance:.2f} m")
            mask_left = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(-10))
            mask_right = (np.isfinite(ranges)) & (np.radians(10) <= angles) & (angles <= np.radians(80))
            left_distance = ranges[mask_left]
            right_distance = ranges[mask_right]
            max_left = max(left_distance)
            max_right = max(right_distance)
            # front_distance -= 1
            angle = np.radians(0)
            if max_left > max_right:
                angle -= np.radians(30)
                if front_distance < 1.0:
                    angle -= np.radians(30)
            else:
                angle += np.radians(30)
                if front_distance < 1.0:
                    angle += np.radians(30)
            front_distance = abs((front_distance-0.75)/np.cos(angle))
            goal_x = front_distance*np.cos(angle)
            goal_y = front_distance*np.sin(angle)
            # goal_y -= 0.5
            # if max_left > max_right:
            #     goal_x -= 1
            # else:
            #     goal_x += 1
        else:
            for i in range(len(x_points)):
                error_x = goal_x - x_points[i]
                error_y = goal_y - y_points[i]
                error = math.sqrt(error_x**2 + error_y**2)
                if error < 0.6:
                    self.get_logger().info(f"🚧 Punto cercano a la pared: ({x_points[i]:.2f}, {y_points[i]:.2f}), error: {error:.2f} m")
                    goal_x *= 0.5
                    goal_y *= 0.5
                    # goal_x += (goal_x - x_points[i]) * 0.5  # Ajuste proporcional
                    # goal_y += (goal_y - y_points[i]) * 0.5
                    break

        # Orientamos el goal hacia el ángulo calculado
        best_angle = math.atan2(goal_y, goal_x)

        # Aseguramos que el ángulo esté en un rango razonable
        if abs(best_angle) > math.radians(90):  # Evitar giros excesivos
            self.get_logger().warn(f"⚠️ Ángulo excesivo de giro ({math.degrees(best_angle):.1f}°). Ajustando...")
            best_angle = np.sign(best_angle) * math.radians(90)

        self.get_logger().info(f"🎯 Nuevo goal: ({goal_x:.2f}, {goal_y:.2f}), yaw: {math.degrees(best_angle):.1f}°")
        return goal_x, goal_y, best_angle

    def rotate_laserscan(self, scan_msg: LaserScan, angle_shift_rad: float) -> LaserScan:
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        shift = int(angle_shift_rad / angle_increment)
        self.get_logger().info("Rotating...")
        # Rota el array circularmente
        rotated_ranges = np.roll(ranges, shift)

        # Crear nuevo mensaje corregido
        corrected_scan = LaserScan()
        corrected_scan.header = scan_msg.header
        corrected_scan.header.frame_id = 'base_link'  # o lo que quieras
        corrected_scan.angle_min = scan_msg.angle_min
        corrected_scan.angle_max = scan_msg.angle_max
        corrected_scan.angle_increment = scan_msg.angle_increment
        corrected_scan.time_increment = scan_msg.time_increment
        corrected_scan.scan_time = scan_msg.scan_time
        corrected_scan.range_min = scan_msg.range_min
        corrected_scan.range_max = scan_msg.range_max
        corrected_scan.ranges = rotated_ranges.tolist()
        corrected_scan.intensities = scan_msg.intensities
        self.get_logger().info("Rotated!")

        return corrected_scan



    def average_lidar_in_blocks(self, ranges, angles, block_size=10):
        # Nos aseguramos de que sean arrays de numpy
        ranges = np.array(ranges)
        angles = np.array(angles)

        # Cortamos para que encaje bien en bloques
        n = len(ranges) - (len(ranges) % block_size)
        ranges = ranges[:n]
        angles = angles[:n]

        # Reshape para hacer bloques
        range_blocks = ranges.reshape(-1, block_size)
        angle_blocks = angles.reshape(-1, block_size)

        # Calculamos promedios por bloque
        avg_ranges = np.mean(range_blocks, axis=1)
        avg_angles = np.mean(angle_blocks, axis=1)

        return avg_ranges, avg_angles

    # Transforma el goal de odom a map y prepara el mensaje para nav2
    def create_and_send_goal(self, x_forward, y_lateral, yaw):
        try:
            x, y, yaw = self.transform_point_base_to_map(x_forward, y_lateral, yaw)
            if x is None or y is None or yaw is None:
                return None, 1
            else:
                goal_map_stamped = PoseStamped()
                goal_map_stamped.header.frame_id = self.map_frame
                goal_map_stamped.header.stamp = self.get_clock().now().to_msg()
                goal_map_stamped.pose.position.x = float(x)
                goal_map_stamped.pose.position.y = float(y)
                goal_map_stamped.pose.orientation.z = math.sin(yaw/2.0)
                goal_map_stamped.pose.orientation.w = math.cos(yaw/2.0)
                self.get_logger().info(f"➡️ Nuevo goal dinámico: ({goal_map_stamped.pose.position.x:.2f}, {goal_map_stamped.pose.position.y:.2f})")
                self.pub_goal.publish(goal_map_stamped)
                return goal_map_stamped, 0

        except Exception as e:
            self.get_logger().warn(f"Error al transformar goal: {e}")
            return None, 1

    # Devuelve si el goal ha sido aceptado o rechazado
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rechazado por Nav2.")
            self.goal_active = False
            return

        self.get_logger().info("✅ Goal aceptado")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # Detecta el resultado del goal
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().warn("🚫 Goal abortado.")
            self.goal_active = False
        elif status == 3:
            self.get_logger().info("🎯 Goal alcanzado (aunque ya se generó otro).")

        self.goal_active = False

    # Transforma de euler a quaternion
    def euler_to_quaternion(self, roll, pitch, yaw):

        # Convertimos Euler (roll, pitch, yaw) a cuaternión
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        return [qx, qy, qz, qw]
    
    def get_yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw  # En radianes

    def transform_point_base_to_map(self, x_base, y_base, yaw_base):
        point_stamped = PoseStamped()
        point_stamped.header.stamp = Time(sec=0, nanosec=0)  # Se puede usar el tiempo actual
        point_stamped.header.frame_id = self.frame_id
        point_stamped.pose.position.x = float(x_base)
        point_stamped.pose.position.y = float(y_base)
        point_stamped.pose.position.z = 0.0
        point_stamped.pose.orientation.z = math.sin(yaw_base/2.0)
        point_stamped.pose.orientation.w = math.cos(yaw_base/2.0)
        # Transformamos el punto al frame map
        try:
            target_frame = 'map'
            transformed_point = self.tf_buffer.transform(
                point_stamped,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().info("Tranformando")
            yaw = self.get_yaw_from_quaternion(transformed_point.pose.orientation.x,
                                               transformed_point.pose.orientation.y,
                                               transformed_point.pose.orientation.z,
                                               transformed_point.pose.orientation.w)
            return transformed_point.pose.position.x, transformed_point.pose.position.y, yaw
        except Exception as e:
            self.get_logger().warn(f"No se pudo transformar: {e}")
            return None, None, None
        
def main(args=None):
    rclpy.init(args=args)
    node = FSM_final()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
