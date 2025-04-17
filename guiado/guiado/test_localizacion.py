import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import yaml

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # Subscripciones y publicadores
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_length = 0.268  # metros

        # Suscribir imagen y calibración
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.publisher = self.create_publisher(Twist, '/aruco_pos', 10)

        # Cargar configuración de ArUcos (posición y orientación en mundo)
        self.aruco_reference = self.load_aruco_config()

    def load_aruco_config(self):
        """
        Carga YAML con estructura:
        arucos:
          <id>:
            position:
              x: <float>
              y: <float>
            rotation: <float>  # radianes
        """
        path = os.path.expanduser('~/Phoenyx_humble/src/guiado/config/Aruco_pos.yaml')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        config = {}
        for id_str, info in data.get('arucos', {}).items():
            marker_id = int(id_str)
            Mx = float(info['position']['x'])
            My = float(info['position']['y'])
            theta = float(info.get('rotation', 0.0))
            config[marker_id] = {'world_x': Mx, 'world_y': My, 'world_yaw': theta}
        return config

    def camera_info_callback(self, msg):
        # Inicializar parámetros intrínsecos
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return  # aún no tenemos calibración
        # Convertir ROS Image a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pose = self.detect_and_localize(frame)
        if pose:
            x, y, yaw = pose
            # Publicar como Twist (linear.x, linear.y, angular.z)
            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = float(y)
            twist.angular.z = float(yaw)
            self.publisher.publish(twist)

    def detect_and_localize(self, frame):
        """
        Detecta ArUco, refina esquinas, estima pose y calcula pose global del robot.
        """
        # Undistort
        h, w = frame.shape[:2]
        new_cam, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w,h), 1)
        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_cam)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is None:
            return None

        # Refinar esquinas
        for c in corners:
            cv2.cornerSubPix(gray, c, winSize=(5,5), zeroZone=(-1,-1),
                             criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))

        # Tomar el primer marcador válido
        for corner, marker_id_arr in zip(corners, ids):
            marker_id = int(marker_id_arr[0])
            # Pose relativa
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, self.marker_length, self.camera_matrix, self.dist_coeffs)
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # Calcular y retornar pose global
            return self.compute_robot_global_pose(tvec, rvec, marker_id)
        return None

    def compute_robot_global_pose(self, tvec, rvec, marker_id):
        """
        Dado tvec=(Xrel,Yrel,Zrel) y rvec del marcador,
        y la pose del marcador en el mundo, devuelve (x,y,yaw) del robot global.
        """
        # Extraer configuración del marcador
        cfg = self.aruco_reference.get(marker_id)
        if cfg is None:
            self.get_logger().warning(f"Marcador {marker_id} no en YAML")
            return None
        Mx, My, thetaM = cfg['world_x'], cfg['world_y'], cfg['world_yaw']

        # 1) Transformación marker→camera
        R_mc, _ = cv2.Rodrigues(rvec)
        t_mc = tvec.reshape(3,1)

        # 2) Invertir: camera→marker
        R_cm = R_mc.T
        t_cm = -R_cm.dot(t_mc).flatten()  # [X_cm, Y_cm, Z_cm]

        # 3) Rotación marker→world (solo plano X-Z)
        c, s = np.cos(thetaM), np.sin(thetaM)
        R_mw = np.array([[c, -s], [s, c]])  # 2×2

        # 4) Offset en X-Z del robot en mundo
        offset = R_mw.dot([t_cm[0], t_cm[2]])
        posXabs = Mx + offset[0]
        posZabs = My + offset[1]

        # 5) Orientación del robot: yaw_world = θM - yaw_cam
        yaw_cam = np.arctan2(R_mc[2,0], R_mc[0,0])
        AngleRobot = ((thetaM - yaw_cam + np.pi) % (2*np.pi)) - np.pi

        return posXabs, posZabs, AngleRobot


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
