#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.timer import Timer
import time
from geometry_msgs.msg import Twist

class ArucoScanPublisher(Node):
    def __init__(self):
        super().__init__('aruco_scan_publisher')

        self.publisher_ = self.create_publisher(
            Bool,
            '/aruco_scan',
            10
        )

        # Publicar True inicialmente
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info('Mensaje publicado en /aruco_scan: False')

        # Esperar 10 segundos y publicar False
        self.create_timer(0.5, self.publish_false_once)

    def publish_false_once(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Mensaje publicado en /aruco_scan: True')

        # Apagar el nodo
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoScanPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
