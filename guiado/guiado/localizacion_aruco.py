import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Se importa el mensaje Bool

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Suscripción para las imágenes y la información de la cámara
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        # Suscripción para activar el procesamiento mediante /aruco_scan
        self.subscription_scan = self.create_subscription(
            Bool,
            '/aruco_scan',
            self.scan_callback,
            10)
        
        # Publicador para la posición resultante
        self.publisher_aruco_pos = self.create_publisher(
            Twist,
            '/aruco_pos',
            10)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_marker_length = 0.3  # No se modifica la longitud del marcador

        # Cargar posiciones de ArUcos desde el archivo YAML
        self.aruco_positions = self.load_aruco_positions()

        # Variables para controlar el disparo de la secuencia y almacenamiento de muestras
        self.active = False
        self.measurements = []  # Lista para almacenar tuples: (posXabs, posZabs, AngleRobot)

    def publish_aruco_position(self, x, y, theta):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(theta)
        self.publisher_aruco_pos.publish(msg)
        self.get_logger().info(f"Publicando posición final: X={x:.3f}, Y={y:.3f}, Ángulo={theta:.3f}")

    def load_aruco_positions(self):
        with open(os.path.expanduser('./src/guiado/config/Aruco_pos.yaml'), 'r') as file:
            aruco_data = yaml.safe_load(file)
        return {aruco['id']: (aruco['position']['x'], aruco['position']['y']) for aruco in aruco_data['arucos']}

    def scan_callback(self, msg):
        if msg.data:  # Si se recibe True (o 1)
            self.get_logger().info("Activación recibida por /aruco_scan. Iniciando proceso de detección en 30 iteraciones.")
            self.active = True
            self.measurements = []  # Reinicia las mediciones

    def image_callback(self, msg):
        if not self.active:
            return  # No se procesa la imagen a menos que esté activado
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # Procesa la imagen para detectar ArUco y estimar la pose.
        result = self.detect_aruco_and_estimate_pose(frame)
        if result is not None:
            self.measurements.append(result)
            self.get_logger().info(f"Medición {len(self.measurements)}/30 obtenida.")
            if len(self.measurements) >= 30:
                # Aplica filtro mediano a cada uno de los valores
                posX_list, posZ_list, angle_list = zip(*self.measurements)
                posX_med = np.median(posX_list)
                posZ_med = np.median(posZ_list)
                angle_med = np.median(angle_list)
                # Publica el resultado único
                self.publish_aruco_position(posX_med, posZ_med, angle_med)
                # Reinicia el proceso para futuras activaciones
                self.active = False
                self.measurements = []

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def undistort_image(self, frame):
        h, w = frame.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

    def detect_aruco_and_estimate_pose(self, frame):
        frame = self.undistort_image(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for corner in corners:
                cv2.cornerSubPix(
                    gray, corner,
                    winSize=(5, 5),
                    zeroZone=(-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for corner, marker_id in zip(corners, ids):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corner, self.aruco_marker_length, self.camera_matrix, self.dist_coeffs)
                
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                self.print_pose(marker_id, tvec, rvec)

                Xrel = tvec[0][0][0]
                Zrel = tvec[0][0][2]

                # ✅ Convertimos el rvec a matriz de rotación y extraemos yaw desde la rotación
                R_mat, _ = cv2.Rodrigues(rvec[0][0])
                thetaArucoRel = np.arctan2(R_mat[2, 0], R_mat[0, 0])  # Esto es yaw (ángulo del robot respecto al ArUco)

                result = self.calculate_robot_pos2(Xrel, Zrel, marker_id[0], thetaArucoRel)
                return result

        return None


    def print_pose(self, marker_id, tvec, rvec):
        self.get_logger().info(
            f"\n=== ArUco Marker Detected ===\nMarker ID: {marker_id[0]}\nTranslation Vector (tvec):\n  X: {tvec[0][0][0]:.3f} m\n  Y: {tvec[0][0][1]:.3f} m\n  Z: {tvec[0][0][2]:.3f} m\nRotation Vector (rvec):\n  Rx: {rvec[0][0][0]:.3f} rad\n  Ry: {rvec[0][0][1]:.3f} rad\n  Rz: {rvec[0][0][2]:.3f} rad")

    def calculate_robot_pos2(self, Xrel, Zrel, aruco_id, thetaArucoRel):
        r = np.hypot(Xrel, Zrel)
        aruco_positions = self.aruco_positions

        # Posición y orientación del ArUco
        xm, ym = aruco_positions[aruco_id]
        thetaArucoAbs = 0.0

        # R_z= [[np.cos(thetaArucoRel), -np.sin(thetaArucoRel), 0], 
            #   [np.sin(thetaArucoRel), np.cos(thetaArucoRel), 0], 
            #   [0, 0, 1]]

        if xm == 0:  # x mínimo
            thetaArucoAbs = 0
        elif xm == 7:  # x máximo
            thetaArucoAbs = np.pi
        elif ym == 0:  # y mínimo
            thetaArucoAbs = np.pi / 2
        elif ym == 7:  # y máximo
            thetaArucoAbs = -np.pi / 2

        
                
        # # Dirección desde el ArUco hacia el robot en marco global
        # phi = np.arctan2(Xrel, Zrel) + np.pi  # porque la cámara ve hacia el marcador
        # alpha = thetaArucoAbs + phi
        # alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        # # Cálculo de posición absoluta con fórmula polar
        # posXabs = xm + r * np.sin(alpha)
        # posZabs = ym + r * np.cos(alpha)

        # Ángulo del robot
        AngleRobot = thetaArucoAbs - thetaArucoRel
        
        thetaArucoRel = np.pi - thetaArucoRel 

        Xabs = xm +  Xrel * np.cos(thetaArucoRel) - Zrel * np.sin(thetaArucoRel)
        Yabs = ym +  Xrel * np.sin(thetaArucoRel) + Zrel * np.cos(thetaArucoRel)

        AngleRobot = (AngleRobot + np.pi) % (2 * np.pi) - np.pi

        self.get_logger().info(f"Medida individual: Posición del robot: X={Xabs:.3f}, Y={Yabs:.3f}, Ángulo={AngleRobot:.3f}")
        
        return Xabs, Yabs, AngleRobot


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
