import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math
import time
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
"""

 ESTO ES SOLO UNA COPIA DE BACKUP


"""
class ContinuousLidarNavigator(Node):
    def __init__(self):
        super().__init__('continuous_lidar_navigator')

        self.goal_distance = 1.0  # distancia hacia adelante
        self.goal_threshold = 2.0  # metros para anticipar siguiente goal
        self.goal_active = False
        self.last_goal_pose = None
        # self.last_goal_angle = None
        self.lidar_msg = None
        # self.clock = self.get_clock()
        self.frame_id = 'base_footprint'
        self.map_frame = 'map'

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_points = self.create_publisher(PointStamped, '/points', 10)
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

        # Timer que llama a check_progress cada 0.5 segundos
        # self.timer = self.create_timer(0.5, self.check_progress)

        self.FSM = self.create_timer(0.1, self.brain)
        self.get_logger().info("⏩ Navegación continua con LiDAR iniciada")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orientation_q = msg.pose.pose.orientation

    # Gestiona la prueba de control
    def brain(self):
        if self.lidar_msg != None:
            if self.goal_active:
                distance = self.check_progress()
                self.get_logger().info(f"Distancia al goal: {distance:.2f} m")
                if distance != -1 and distance < self.goal_threshold:
                    self.get_logger().info(f"📍 Cerca del goal ({distance:.2f} m)")
                    self.goal_active = False
            # else:

                
                
    # Actualiza el mensaje del lidar
    def lidar_callback(self, msg):
        self.lidar_msg = msg
        if not self.goal_active:
            self.get_logger().info("Generando siguiente goal...")
            x_forward, y_lateral, yaw = self.generate_goal_from_lidar(self.lidar_msg)
            goal, error = self.create_and_send_goal(x_forward, y_lateral, yaw)
            self.goal_active = not error
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
        if front_distance < 1.5:
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
                angle -= np.radians(45)
            else:
                angle += np.radians(45)
            
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
        # Comprobamos si el goal está demasiado cerca de cualquier punto
        # closest_distance = min(valid_ranges)
        # if closest_distance < 1.0:  # Si la distancia más cercana está por debajo de 1 metro
        #     self.get_logger().warn(f"🚧 Goal demasiado cercano a una pared, ajustando...")

        #     # Calcular el vector de dirección del goal (dirección media)
        #     goal_angle = math.atan2(goal_y, goal_x)

        #     # Aplicar un offset hacia el robot
        #     # Si el goal está cerca, acercamos el goal hacia el robot
        #     offset_factor = 0.5  # Factor de cuánto nos acercamos al robot
        #     adjusted_distance = max(0.5, closest_distance - offset_factor)  # No permitimos que el goal se acerque más de 0.5 metros

        #     # Calculamos el nuevo goal
        #     goal_x = adjusted_distance * math.cos(goal_angle)
        #     goal_y = adjusted_distance * math.sin(goal_angle)

        # Orientamos el goal hacia el ángulo calculado
        best_angle = math.atan2(goal_y, goal_x)

        # Aseguramos que el ángulo esté en un rango razonable
        if abs(best_angle) > math.radians(90):  # Evitar giros excesivos
            self.get_logger().warn(f"⚠️ Ángulo excesivo de giro ({math.degrees(best_angle):.1f}°). Ajustando...")
            best_angle = np.sign(best_angle) * math.radians(90)

        self.get_logger().info(f"🎯 Nuevo goal: ({goal_x:.2f}, {goal_y:.2f}), yaw: {math.degrees(best_angle):.1f}°")

        return goal_x, goal_y, best_angle

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
            # while not self.tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10.0)):
            #     self.get_logger().warn("Esperando transformación entre 'map' y 'base_footprint'...")
            #     time.sleep(0.1)
            # now = rclpy.time.Time()
            # transform_base_to_odom = self.tf_buffer.lookup_transform(
            #     "map", "odom", now, timeout=rclpy.duration.Duration(seconds=10.0)
            # )
            # # transform_odom_to_map = self.tf_buffer.lookup_transform(
            # #     self.map_frame, 'odom', now, timeout=rclpy.duration.Duration(seconds=5.0)
            # # )

            # goal = PoseStamped()
            # goal.header.frame_id = "base_footprint"
            # goal.header.stamp = now.to_msg()

            # goal.pose.position.x = float(x_forward)
            # goal.pose.position.y = float(y_lateral)
            # goal.pose.position.z = 0.0

            # q = self.euler_to_quaternion(0, 0, yaw)  # usamos el yaw calculado
            # goal.pose.orientation.x = float(q[0])
            # goal.pose.orientation.y = float(q[1])
            # goal.pose.orientation.z = float(q[2])
            # goal.pose.orientation.w = float(q[3])

            # goal_odom = tf2_geometry_msgs.do_transform_pose(goal.pose, transform_base_to_odom)
            # goal_map = tf2_geometry_msgs.do_transform_pose(goal_odom, transform_odom_to_map)
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
        

    # def transform_point_base_to_map(self, x_base, y_base, yaw_base):
    #     """Error al transformar

    #     Transforma un punto (x, y) en base_footprint al frame map usando odometría pura.

    #     Args:
    #         x_base, y_base: Coordenadas del punto en base_footprint
    #         odom_msg: Mensaje de tipo nav_msgs.msg.Odometry

    #     Returns:yaw/2.0
    #         (x_map, y_map): Punto transformado al frame map
    #     """
    #     # Posición del robot en el mapa
    #     x_map_robot = self.x
    #     y_map_robot = self.y

    #     # Orientación del robot como cuaternión → yaw
    #     q = self.orientation_q
    #     if q is None:
    #         return
    #     siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)

    #     # Rotación + traslación
    #     x_rot = x_base * math.cos(yaw) - y_base * math.sin(yaw)
    #     y_rot = x_base * math.sin(yaw) + y_base * math.cos(yaw)

    #     x_map = x_map_robot + x_rot
    #     y_map = y_map_robot + y_rot

    #     theta_map = yaw + yaw_base

    #     # Normalizamos el ángulo a [-pi, pi]
    #     theta_map = math.atan2(math.sin(theta_map), math.cos(theta_map))

    #     return x_map, y_map, theta_map


def main():
    rclpy.init()
    node = ContinuousLidarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
