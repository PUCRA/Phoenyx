import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class RecorteSubscriber(Node):
    def __init__(self):
        super().__init__('recorte_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/recorte',
            self.image_callback,
            10)
        self.subscription2 = self.create_subscription(
            Image,
            '/recorte_bin',
            self.image_callback2,
            10)
        self.subscription3 = self.create_subscription(
            Image,
            '/recorte_bin_2',
            self.image_callback3,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Mostrar la imagen
            cv2.imshow("Recorte recibido", cv_image)
            cv2.waitKey(1)  # Refrescar la ventana
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")
    
    def image_callback2(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Mostrar la imagen
            cv2.imshow("Recorte binarizado", cv_image)
            cv2.waitKey(1)  # Refrescar la ventana
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")
    def image_callback3(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Mostrar la imagen
            cv2.imshow("Recorte binarizado 2", cv_image)
            cv2.waitKey(1)  # Refrescar la ventana
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RecorteSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Cerrar ventanas OpenCV al salir

if __name__ == '__main__':
    main()
