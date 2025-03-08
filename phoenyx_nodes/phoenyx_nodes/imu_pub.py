import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno055 import BNO055_I2C
from sensor_msgs.msg import Imu  # 📌 Cambiado de Twist a Imu
import numpy as np

class IMU_Publisher(Node):

    def __init__(self):
        super().__init__('IMU')
        self.get_logger().info("IMU node started")
        self.create_timer(0.02, self.timer_callback)

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO055_I2C(i2c, address=0x28)
        except Exception as e:
            self.get_logger().error(f"Error initializing IMU: {e}")
            return

        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.frame_id = self.declare_parameter('frame_id', "imu_link").value

    def timer_callback(self):
        try:
            imu_msg = Imu()

            # Timestamp
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Orientación (usando quaternion en lugar de Euler)
            quaternion = self.bno.quaternion
            if quaternion:
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]

            # Velocidad angular
            gyro = self.bno.gyro
            if gyro:
                imu_msg.angular_velocity.x = gyro[0]
                imu_msg.angular_velocity.y = gyro[1]
                imu_msg.angular_velocity.z = gyro[2]

            # Aceleración lineal
            acceleration = self.bno.acceleration
            if acceleration:
                imu_msg.linear_acceleration.x = acceleration[0]
                imu_msg.linear_acceleration.y = acceleration[1]
                imu_msg.linear_acceleration.z = acceleration[2]

            self.pub.publish(imu_msg)
        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = IMU_Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
