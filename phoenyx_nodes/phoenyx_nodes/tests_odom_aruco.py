import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class tests_aruco_odom(Node):
    def __init__(self):
        super().__init__('tests_odom_aruco.py')
        #creamos subscirptores y publishers
        self.create_subscription(Twist, '/aruco_pose',self.aruco_pose_callback ,10)
        self.create_subscription(Joy, '/joy', self.joy_callback ,10)
        #publishers
        self.publisher_ = self.create_publisher(Bool, '/aruco_scan', 10)
    
    def joy_callback(self):
        msg=Bool()
        msg.data = True
        self.publisher_.publish(msg)
        msg.data = False
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TestsArucoOdom(Node):
    def __init__(self):
        super().__init__('tests_odom_aruco')

        # Suscriptores
        self.create_subscription(Twist, '/aruco_pose', self.aruco_pose_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher
        self.publisher_ = self.create_publisher(Bool, '/aruco_scan', 10)

    def aruco_pose_callback(self, msg):
        # Acá podrías hacer algo con el mensaje de aruco_pose
        self.get_logger().info('Recibido aruco_pose')

    def joy_callback(self, msg):
        # Simula un escaneo al presionar un botón del joystick, por ejemplo el botón 0
        if msg.buttons[0] == 1:
            scan_msg = Bool()
            scan_msg.data = True
            self.publisher_.publish(scan_msg)
            self.get_logger().info('Publicando True en /aruco_scan')

            # Si querés enviar False inmediatamente después
            scan_msg.data = False
            self.publisher_.publish(scan_msg)
            self.get_logger().info('Publicando False en /aruco_scan')

def main(args=None):
    rclpy.init(args=args)
    node = TestsArucoOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


