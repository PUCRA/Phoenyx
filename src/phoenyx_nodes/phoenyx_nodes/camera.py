import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import yaml
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
import os

class USBImagePublisher(Node):

    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Obtener el directorio del paquete
        package_share_dir = get_package_share_directory('phoenyx_nodes')  # Reemplazar 'tu_paquete' con el nombre de tu paquete
        config_file_path = os.path.join(package_share_dir, 'conf', 'camara_params.yaml')  # Ruta al archivo YAML en el directorio 'config'

        self.config = self.load_config(config_file_path)

        # Configuración de la cámara desde el YAML
        self.width = self.config['camera']['width']
        self.height = self.config['camera']['height']
        self.fps = self.config['camera']['fps']
        self.exposure = self.config['camera']['exposure']
        self.color_gain = self.config['camera']['color_gain']
        self.brightness = self.config['camera']['brightness']
        self.contrast = self.config['camera']['contrast']

        # Abrir la cámara USB
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, self.color_gain)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)

        # Publicador de imágenes
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Temporizador para publicar las imágenes a 30fps
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def load_config(self, config_file):
        # Cargar los parámetros desde el archivo YAML
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)

    def timer_callback(self):
        # Leer un nuevo frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo capturar imagen")
            return

        # Convertir la imagen a formato ROS
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info("Imagen publicada")
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = USBImagePublisher()
    rclpy.spin(node)

    # Al salir, liberar recursos
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
