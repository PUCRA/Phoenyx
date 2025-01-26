
import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno055 import BNO055_I2C
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

class IMU_Publisher(Node):
    
    def __init__(self):
        super().__init__('IMU')
        self.get_logger().info("IMU node started")
        self.create_timer(1, self.timer_callback)
        # Configuración del bus I2C
        i2c = busio.I2C(board.SCL, board.SDA)

        # Inicializar el sensor
        self.bno = BNO055_I2C(i2c, address=0x28)
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.frame_id = self.declare_parameter('frame_id', "base_imu_link").value

    def timer_callback(self):
        """Publish the sensor message with new data
        """
        try:
            imu_msg = Imu()
            gyro = Vector3()
            gyro.x, gyro.y, gyro.z = self.bno.gyro
            self.get_logger().info(f"Now gathering data for message")

            # fetch all accel values - return in m/s²
            accel = Vector3()
            accel.x, accel.y, accel.z = self.bno.acceleration

            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance[0] = 0.00001
            imu_msg.angular_velocity_covariance[4] = 0.00001
            imu_msg.angular_velocity_covariance[8] = 0.00001

            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance[0] = 0.00001
            imu_msg.linear_acceleration_covariance[4] = 0.00001
            imu_msg.linear_acceleration_covariance[8] = 0.00001

            # imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            imu_msg.orientation.w = quat_real
            imu_msg.orientation.x = quat_i
            imu_msg.orientation.y = quat_j
            imu_msg.orientation.z = quat_k

            imu_msg.orientation_covariance[0] = 0.00001
            imu_msg.orientation_covariance[4] = 0.00001
            imu_msg.orientation_covariance[8] = 0.00001

            # add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu'

            self.get_logger().info('Publishing imu message')
            #self.pub_imu_raw.publish(imu_raw_msg)
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
