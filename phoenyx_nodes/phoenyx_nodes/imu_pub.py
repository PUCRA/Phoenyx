
import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno055 import BNO055_I2C
from geometry_msgs.msg import Twist

class IMU_Publisher(Node):
    
    def __init__(self):
        super().__init__('IMU')
        self.get_logger().info("IMU node started")
        self.create_timer(0.02, self.timer_callback)
        # Configuración del bus I2C
        i2c = busio.I2C(board.SCL, board.SDA)

        # Inicializar el sensor
        self.bno = BNO055_I2C(i2c, address=0x28)
        self.pub = self.create_publisher(Twist, 'imu/data', 10)
        self.frame_id = self.declare_parameter('frame_id', "base_imu_link").value
        self.gyro = Twist()
        self.gyro.linear.x, self.gyro.linear.y, self.gyro.linear.z = 0.0, 0.0, 0.0
        self.gyro.angular.x, self.gyro.angular.y, self.gyro.angular.z = 0.0, 0.0, 0.0


    def timer_callback(self):
        """Publish the sensor message with new data
        """
        try:
            try:
                self.gyro.linear.x, self.gyro.linear.y, self.gyro.linear.z = self.bno.euler #euler
                self.gyro.angular.x, self.gyro.angular.y, self.gyro.angular.z = self.bno.acceleration
                # Imprimir en logs para ver qué está pasando
                #self.get_logger().info(f"Euler angles: {self.bno.euler}")
                #self.get_logger().info(f"Acceleration: {self.bno.acceleration}")
            except Exception as e:
                # gyro.x, gyro.y, gyro.z = 0, 0, 0
                self.get_logger().warning('No data from IMU')
            #self.get_logger().info(f"Now gathering data for message")

            # add header

            #self.get_logger().info('Publishing imu message')
            #self.pub_imu_raw.publish(imu_raw_msg)
            self.pub.publish(self.gyro)

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
