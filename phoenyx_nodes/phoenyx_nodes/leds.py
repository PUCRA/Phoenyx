import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')

        # Configura el puerto serie
        self.serial_port = serial.Serial('/dev/arduino', 9600, timeout=1)

        # Últimos valores
        self.moving = False
        self.voltage_alert = False

        # Subscripciones
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

        # Timer para evaluar y enviar estado cada 1 segundo
        self.create_timer(1.0, self.evaluate_state)

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        self.moving = abs(linear) > 0.01 or abs(angular) > 0.01
        self.get_logger().debug(f'Moving: {self.moving}')

    def battery_callback(self, msg: BatteryState):
        voltage = msg.voltage
        self.voltage_alert = voltage <= 14.8 or voltage >= 16.8
        self.get_logger().debug(f'Voltage: {voltage} | Alert: {self.voltage_alert}')

    def evaluate_state(self):
        if self.voltage_alert:
            estado, fase = 3, 0  # Rojo - parpadeo
        elif self.moving:
            estado, fase = 2, 2  # Amarillo - secuencial
        else:
            estado, fase = 0, 1  # Verde - constante

        comando = f"{estado}{fase}\n"
        self.serial_port.write(comando.encode())
        # self.get_logger().info(f'Enviado a Arduino: {comando.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = LedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
