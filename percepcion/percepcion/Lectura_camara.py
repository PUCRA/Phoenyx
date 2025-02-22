import rclpy
from rclpy.node import Node
from percepcion.Img2recorte import image2recorte
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from message_filters import ApproximateTimeSynchronizer
import yaml
import os
import numpy as np


class LecturaCamara(Node):
    def __init__(self):
        super().__init__('lectura_camara')
        # Parámetros de la cámara
        self.declare_parameters(namespace='', parameters=[
            ('depth_filter.min', 1000), 
            ('depth_filter.max', 2000),
        ])

        # Leer los parámetros
        self.min_depth = self.get_parameter('depth_filter.min').get_parameter_value().integer_value
        self.max_depth = self.get_parameter('depth_filter.max').get_parameter_value().integer_value
        self.get_logger().info('Rango de profundidad: %d - %d' % (self.min_depth, self.max_depth))
        # Inicialización de la cámara
        self.converter = image2recorte()
        self.br = CvBridge()
        self.color_subscription = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_subscription = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        self.pub = self.create_publisher(Image, 'percepcion/recorte', 10)
        # self.pub2 = self.create_publisher(CompressedImage, 'percepcion/recorte_compressed', 10)

        # Sincronizador de los dos tópicos con una tolerancia en el tiempo
        self.ts = ApproximateTimeSynchronizer(
            [self.color_subscription, self.depth_subscription], 
            queue_size=10, 
            slop=0.1  # Tiempo máximo de diferencia entre los mensajes para ser sincronizados
        )
        self.ts.registerCallback(self.listener_callback)
        self.get_logger().info('Lectura de cámara iniciada')
        self.i = 0

    def listener_callback(self, color_msg, depth_msg):
        try:
            # Convertir las imágenes de ROS a formato OpenCV
            color_image = self.br.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            # Filtrar los píxeles de color basándonos en el rango de profundidad
            # Definimos un rango de valores de profundidad (en milímetros)

            # Crear una máscara donde la profundidad está dentro del rango especificado
            mask = (depth_image >= self.min_depth) & (depth_image <= self.max_depth)

            # Crear una imagen filtrada de color con los píxeles que están dentro del rango de profundidad
            filtered_color_image = color_image.copy()
            filtered_color_image[~mask] = 0  # Ponemos en negro los píxeles fuera del rango
            # Mostrar la imagen filtrada de color
            # cv2.imshow("Filtered Color Image", filtered_color_image)
            recorte = self.converter.obtener_recorte(filtered_color_image)
            if recorte is not None:
                # cv2.imshow("Recorte", recorte)
                imagen_redimensionada = cv2.resize(recorte, (28, 28), interpolation=cv2.INTER_LINEAR)
                ros_image = self.br.cv2_to_imgmsg(imagen_redimensionada, encoding='bgr8')
                self.get_logger().info("Enviando recorte ")
                # self.pub2.publish(ros_image2)
                self.pub.publish(ros_image)


        except Exception as e:
            self.get_logger().error('Error al procesar las imágenes: %s' % str(e))



def main(args=None):
    rclpy.init(args=args)
    lectura_camara = LecturaCamara()
    rclpy.spin(lectura_camara)
    lectura_camara.destroy_node()
    rclpy.shutdown()