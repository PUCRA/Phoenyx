import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from percepcion.pid import pid
import time
import math

class DarVueltas(Node):
    def __init__(self):
        super().__init__('dar_vueltas')
        self.subscriber_ = self.create_subscription(Int32, '/num_vueltas', self.callback, 10)
        self.subs_imu = self.create_subscription(Twist, '/imu', self.imu_update, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controlador = pid(1, 0, 0, 0)
        self.controlador.set_max_val(0.5)
        self.rotation = 0
        self.num_vueltas = 0

    def callback(self, msg):
        if self.num_vueltas == 0 and msg.data != 0:
            self.get_logger().info('Dando {} vueltas'.format(msg.data))
            self.num_vueltas = msg.data
            self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Ángulo actual: {}'.format(self.rotation))
        self.controlador.set_setpoint(self.num_vueltas*2*math.pi)
        value = self.controlador.update(self.rotation, 0.02)
        self.get_logger().info('Valor de control: {}'.format(value))
        Twist_msg = Twist()
        Twist_msg.angular.z = value
        Twist_msg.linear.x = 0.0
        if abs(value) < 0.1:
            self.get_logger().info('Deteniendo robot')
            self.num_vueltas = 0
            Twist_msg.angular.z = 0.0
            self.timer.destroy()
        self.get_logger().info('Publicando mensaje')
        self.pub.publish(Twist_msg)

    def imu_update(self, msg):
        self.get_logger().info('IMU data: {}'.format(msg))
        delta_yaw = (msg.angular.z - self.prev_angle)
        if delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi
        self.rotation += (msg.angular.z-self.prev_angle)
        self.prev_angle = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    dar_vueltas = DarVueltas()
    rclpy.spin(dar_vueltas)
    dar_vueltas.destroy_node()
    rclpy.shutdown()