import rclpy
from rclpy.node import Node
from osr_interfaces.msg import CommandDrive as CmdDrive # Asegúrate de usar tu paquete y nombre de mensaje

class CmdDrivePublisher(Node):
    def __init__(self):
        super().__init__('cmd_drive_publisher')
        self.publisher_ = self.create_publisher(CmdDrive, '/cmd_drive', 10)
        self.get_logger().info('Nodo inicializado. Introduce velocidades para los motores.')

        # Solicitar al usuario las velocidades y publicarlas
        self.publish_cmd_drive()

    def publish_cmd_drive(self):
        while rclpy.ok():
            try:
                self.get_logger().info('Introduce las velocidades en rad/s:')
                left_front_vel = float(input('Velocidad izquierda delantera: '))
                left_middle_vel = float(input('Velocidad izquierda central: '))
                left_back_vel = float(input('Velocidad izquierda trasera: '))
                right_front_vel = float(input('Velocidad derecha delantera: '))
                right_middle_vel = float(input('Velocidad derecha central: '))
                right_back_vel = float(input('Velocidad derecha trasera: '))

                # Crear el mensaje y rellenar los valores
                cmd_drive_msg = CmdDrive()
                cmd_drive_msg.left_front_vel = left_front_vel
                cmd_drive_msg.left_middle_vel = left_middle_vel
                cmd_drive_msg.left_back_vel = left_back_vel
                cmd_drive_msg.right_front_vel = right_front_vel
                cmd_drive_msg.right_middle_vel = right_middle_vel
                cmd_drive_msg.right_back_vel = right_back_vel

                # Publicar el mensaje
                self.publisher_.publish(cmd_drive_msg)
                self.get_logger().info(f'Publicado: {cmd_drive_msg}')
            except ValueError:
                self.get_logger().error('Por favor, introduce valores válidos.')

def main(args=None):
    rclpy.init(args=args)
    node = CmdDrivePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
