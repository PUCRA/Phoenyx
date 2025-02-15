import rclpy
from rclpy.node import Node
from percepcion.Img2number import image2number
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from message_filters import ApproximateTimeSynchronizer

class LecturaCamara(Node):
    def __init__(self):
        super().__init__('lectura_camara')
        # Parámetros de la cámara
        index = 0
        exposicion = -5
        ganancia = 1
        brillo = 50
        contraste = 85
        
        # Inicialización de la cámara
        self.converter = image2number()
        self.br = CvBridge()
        self.color_subscription = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_subscription = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')

        # Sincronizador de los dos tópicos con una tolerancia en el tiempo
        self.ts = ApproximateTimeSynchronizer(
            [self.color_subscription, self.depth_subscription], 
            queue_size=10, 
            slop=0.1  # Tiempo máximo de diferencia entre los mensajes para ser sincronizados
        )
        self.ts.registerCallback(self.listener_callback)
        self.get_logger().info('Lectura de cámara iniciada')

    def listener_callback(self, color_msg, depth_msg):
        try:
            # Convertir las imágenes de ROS a formato OpenCV
            color_image = self.br.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            # Filtrar los píxeles de color basándonos en el rango de profundidad
            # Definimos un rango de valores de profundidad (en milímetros)
            min_depth = 1000  # 1 metro
            max_depth = 2000  # 3 metros

            # Crear una máscara donde la profundidad está dentro del rango especificado
            mask = (depth_image >= min_depth) & (depth_image <= max_depth)

            # Crear una imagen filtrada de color con los píxeles que están dentro del rango de profundidad
            filtered_color_image = color_image.copy()
            filtered_color_image[~mask] = 0  # Ponemos en negro los píxeles fuera del rango

            # Mostrar la imagen filtrada de color
            cv2.imshow("Filtered Color Image", filtered_color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error('Error al procesar las imágenes: %s' % str(e))


    def close_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    lectura_camara = LecturaCamara()
    rclpy.spin(lectura_camara)
    lectura_camara.close_camera()
    lectura_camara.destroy_node()
    rclpy.shutdown()