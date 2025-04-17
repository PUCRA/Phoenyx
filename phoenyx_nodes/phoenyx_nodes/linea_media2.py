import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math

class ContinuousLidarNavigator(Node):
    def __init__(self):
        super().__init__('continuous_lidar_navigator')

        self.goal_distance = 2.0  # distancia hacia adelante
        self.goal_threshold = 1.0  # metros para anticipar siguiente goal
        self.goal_active = False
        self.last_goal_pose = None
        # self.clock = self.get_clock()
        self.frame_id = 'base_footprint'
        self.map_frame = 'map'

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer que llama a check_progress cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.check_progress)

        self.get_logger().info("⏩ Navegación continua con LiDAR iniciada")

    def lidar_callback(self, msg):
        if self.goal_active:
            return

        self.generate_goal_from_lidar(msg)

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
        x_forward = 3.0
        y_lateral = (left_distance-right_distance)

        self.create_and_send_goal(x_forward, y_lateral)

    def create_and_send_goal(self, x_forward, y_lateral):
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
            self.send_goal_to_nav2(goal_map_stamped)

        except Exception as e:
            self.get_logger().warn(f"Error al transformar goal: {e}")

    
    
    def send_goal_to_nav2(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.last_goal_pose = pose

        self.get_logger().info(f"➡️ Nuevo goal dinámico: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.nav_client.wait_for_server()

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rechazado por Nav2.")
            self.goal_active = False
            return

        self.get_logger().info("✅ Goal aceptado")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().warn("🚫 Goal abortado.")
        elif status == 3:
            self.get_logger().info("🎯 Goal alcanzado (aunque ya se generó otro).")

        self.goal_active = False

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
