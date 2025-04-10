import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math
import tf2_ros
import tf2_geometry_msgs

class ContinuousLidarNavigator(Node):
    def __init__(self):
        super().__init__('continuous_lidar_navigator')

        self.goal_distance = 2.0  # distancia hacia adelante
        self.goal_threshold = 1.0  # metros para anticipar siguiente goal
        self.goal_active = False
        self.last_goal_pose = None
        self.frame_id = 'base_link'
        self.map_frame = 'map'

        # Crear un publisher para el goal en el topic '/goal_pose'
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer que llama a check_progress cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.check_progress)

        self.get_logger().info("⏩ Navegación continua con LiDAR iniciada")
        self.first_time = True

    def lidar_callback(self, msg):
        if self.first_time:
            self.first_time = False
            self.generate_goal_from_lidar(msg)
        else:
            num_readings = len(msg.ranges)
            left_index = 1
            right_index = num_readings // 2
            left_distance = msg.ranges[left_index]
            right_distance = msg.ranges[right_index]
            middle_error = left_distance - right_distance
            self.get_logger().info(f"Distancia izquierda: {left_distance:.2f}, derecha: {right_distance:.2f}, error medio: {middle_error:.2f}")

    def check_progress(self):
        if not self.goal_active or self.last_goal_pose is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            goal_x = self.last_goal_pose.pose.position.x
            goal_y = self.last_goal_pose.pose.position.y

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

            if distance < self.goal_threshold:
                self.get_logger().info(f"📍 Cerca del goal ({distance:.2f} m) → Generando siguiente...")
                self.goal_active = False  # Permite que lidar_callback lo regenere

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"[TF Error] al verificar progreso: {e}")

    def generate_goal_from_lidar(self, msg):
        num_readings = len(msg.ranges)
        left_index = 1
        right_index = num_readings // 2
        left_distance = msg.ranges[left_index]
        right_distance = msg.ranges[right_index]

        # Calculamos la nueva posición del goal
        x_forward = 3.4
        y_lateral = (-left_distance + right_distance)

        self.create_and_publish_goal(x_forward, y_lateral)

    def create_and_publish_goal(self, x_forward, y_lateral):
        try:
            now = rclpy.time.Time()

            transform_base_to_odom = self.tf_buffer.lookup_transform(
                'odom', self.frame_id, now, timeout=rclpy.duration.Duration(seconds=2.0)
            )
            transform_odom_to_map = self.tf_buffer.lookup_transform(
                self.map_frame, 'odom', now, timeout=rclpy.duration.Duration(seconds=2.0)
            )

            goal = PoseStamped()
            goal.header.frame_id = self.frame_id
            goal.header.stamp = self.get_clock().now().to_msg()

            goal.pose.position.x = float(x_forward)
            goal.pose.position.y = float(y_lateral)
            goal.pose.position.z = 0.0
            yaw = math.atan2(y_lateral, x_forward)  # calculamos el yaw
            q = self.euler_to_quaternion(0, 0, yaw)  # usamos el yaw calculado
            self.get_logger().info(f"yaw: {yaw}")
            goal.pose.orientation.x = float(q[0])
            goal.pose.orientation.y = float(q[1])
            goal.pose.orientation.z = float(q[2])
            goal.pose.orientation.w = float(q[3])

            goal_odom = tf2_geometry_msgs.do_transform_pose(goal.pose, transform_base_to_odom)
            goal_map = tf2_geometry_msgs.do_transform_pose(goal_odom, transform_odom_to_map)
            goal_map_stamped = PoseStamped()
            goal_map_stamped.header.frame_id = self.map_frame
            goal_map_stamped.header.stamp = self.get_clock().now().to_msg()
            goal_map_stamped.pose = goal_map

            # Publicamos el goal en el topic '/goal_pose'
            self.publish_goal(goal_map_stamped)

        except Exception as e:
            self.get_logger().warn(f"Error al transformar goal: {e}")
            self.first_time = True

    def publish_goal(self, goal):
        self.get_logger().info(f"➡️ Publicando nuevo goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        self.goal_publisher.publish(goal)
        self.last_goal_pose = goal

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convertimos Euler (roll, pitch, yaw) a cuaternión
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        return [qx, qy, qz, qw]

def main():
    rclpy.init()
    node = ContinuousLidarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
