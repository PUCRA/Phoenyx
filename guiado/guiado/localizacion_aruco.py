import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge        
import numpy as np
import cv2
import os
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Se importa el mensaje Bool

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.simulation = False
        # self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_marker_length = 0.243  # No se modifica la longitud del marcador

        if self.simulation:
            # Modo simulación
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
            
        else:
            # Modo real: abrimos la cámara y cargamos calibración de ficheros
            calib_dir = os.path.expanduser('./src/phoenyx_nodes/scripts_malosh/aruco/calib_params')
            res = "720p"
            cam_mat_file = os.path.join(calib_dir, f'camera_matrix_{res}.npy')
            dist_file    = os.path.join(calib_dir, f'dist_coeffs_{res}.npy')
            self.camera_matrix = np.load(cam_mat_file)
            self.dist_coeffs   = np.load(dist_file)
            # Inicializa VideoCapture
            self.cap = cv2.VideoCapture(0)
            w, h = (1280, 720) if res=="720p" else (640,480)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            # Timer para leer frame a frame (ej. 30 Hz)
            fps = 10.0
            self.create_timer(1.0/fps, self.timer_callback)

        
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
        

        # Cargar posiciones de ArUcos desde el archivo YAML
        self.aruco_positions = self.load_aruco_positions()

        # Variables para controlar el disparo de la secuencia y almacenamiento de muestras
        self.active = False
        self.measurements = []  # Lista para almacenar tuples: (posXabs, posZabs, AngleRobot)

    def publish_aruco_position(self, x, y, theta):
        msg = Twist()
        msg.linear.x = float(x-1)
        msg.linear.y = float(y-1)
        msg.angular.z = float(theta)
        self.publisher_aruco_pos.publish(msg)
        self.get_logger().info(f"Publicando posición final msg: X={msg.linear.x:.3f}, Y={msg.linear.y:.3f}, Ángulo={msg.angular.z:.3f}")

    def load_aruco_positions(self):
        with open(os.path.expanduser('./src/guiado/config/Aruco_pos.yaml'), 'r') as file:
            aruco_data = yaml.safe_load(file)
        return {aruco['id']: (aruco['position']['x'], aruco['position']['y'], aruco['orientation']) for aruco in aruco_data['arucos']}

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
                # thetaArucoRel = np.arctan2(R_mat[2, 0], R_mat[0, 0])  # Esto es yaw (ángulo del robot respecto al ArUco)

                result = self.calculate_robot_pos2(tvec, R_mat, marker_id[0])
                return result

        return None


    def print_pose(self, marker_id, tvec, rvec):
        self.get_logger().info(
            f"\n=== ArUco Marker Detected ===\nMarker ID: {marker_id[0]}\nTranslation Vector (tvec):\n  X: {tvec[0][0][0]:.3f} m\n  Y: {tvec[0][0][1]:.3f} m\n  Z: {tvec[0][0][2]:.3f} m\nRotation Vector (rvec):\n  Rx: {rvec[0][0][0]:.3f} rad\n  Ry: {rvec[0][0][1]:.3f} rad\n  Rz: {rvec[0][0][2]:.3f} rad")

    def calculate_robot_pos2(self, tvec, R_mat, aruco_id):
        x_aruco_mapa, z_aruco_mapa, theta_aruco_mapa = self.aruco_positions[aruco_id]
        self.get_logger().info(f"X_aruco: {x_aruco_mapa} Y_aruco: {z_aruco_mapa}, theta: {theta_aruco_mapa}")
        T = tvec[0][0].reshape((3, 1))       # traslación del ArUco respecto a la cámara
        R_inv = R_mat.T                         # Rotación inversa
        T_inv = -np.dot(R_inv, T)          # Traslación inversa
        # Posición del robot respecto al aruco
        z_rel = T_inv[0, 0] + 0.15 # sumamos offset posicion camara
        x_rel = T_inv[2, 0]

        # Rotamos e insertamos al sistema del mapa
        cos_theta = np.cos(theta_aruco_mapa)
        sin_theta = np.sin(theta_aruco_mapa)

        xrel = (cos_theta * x_rel - sin_theta * z_rel)
        yrel = (sin_theta * x_rel + cos_theta * z_rel)
        self.get_logger().info(f"Xarcuo robot: {xrel} Yaruco robot: {yrel}")
        Xabs = x_aruco_mapa - xrel
        Yabs = z_aruco_mapa - yrel
        yaw_rel = np.arctan2(R_mat[2, 0], R_mat[0, 0])  # orientación de la cámara en el marco del ArUco
        AngleRobot = theta_aruco_mapa - yaw_rel
        AngleRobot=(AngleRobot + np.pi) % (2 * np.pi) - np.pi #aqui normalizamos el angulo 

        
        return Xabs, Yabs, AngleRobot

    def timer_callback(self):
        if not self.active:
            # self.cap.release()
            return  # No se procesa la imagen a menos que esté activado
                 

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo leer frame de la cámara real")
            return

        result = self.detect_aruco_and_estimate_pose(frame)
        if result is not None:
            self.measurements.append(result)
            self.get_logger().info(f"Medición {len(self.measurements)}/10 obtenida.")
            if len(self.measurements) >= 10:
                posX_list, posZ_list, angle_list = zip(*self.measurements)
                posX_med = np.median(posX_list)
                posZ_med = np.median(posZ_list)
                angle_med = np.median(angle_list)
                self.publish_aruco_position(posX_med, posZ_med, angle_med)
                self.active = False
                self.measurements = []



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

