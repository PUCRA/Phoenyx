import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class FrontDistanceNode(Node):
    def __init__(self):        
        super().__init__('front_distance_node')
        self.get_logger().info("Nodo iniciado")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        self.subscription  # Para evitar que el GC la elimine

    def laser_callback(self, msg):
        num_readings = len(msg.ranges)

        # Índices: inicio (0) y centro
        left_index = 1
        right_index = num_readings // 2

        left_distance = msg.ranges[left_index]
        center_distance = msg.ranges[right_index]

        total_distance = left_distance + center_distance
        error = left_distance - center_distance


        self.get_logger().info(f'Distancia total: {total_distance:.2f} m')
        self.get_logger().info(f'Error: {error:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = FrontDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
