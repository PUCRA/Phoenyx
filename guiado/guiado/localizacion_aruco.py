import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import yaml

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
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
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_marker_length = 0.268  # 0.268 m y 0.174 cm

        # Cargar posiciones de ArUcos desde el archivo YAML
        self.aruco_positions = self.load_aruco_positions()

    def load_aruco_positions(self):
        with open(os.path.expanduser('~/Phoenyx_sym/src/guiado/config/Aruco_pos.yaml'), 'r') as file:
            aruco_data = yaml.safe_load(file)
        # Ruta a los archivos de calibración
        # calibration_path = os.path.expanduser("~/Phoenyx/src/")
        # return {aruco['id']: (aruco['position']['x'], aruco['position']['y']) for aruco in aruco_data['arucos']}

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_with_markers = self.detect_aruco_and_estimate_pose(frame)
        # Aquí puedes publicar o mostrar el frame procesado si es necesario

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
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.aruco_marker_length, self.camera_matrix, self.dist_coeffs)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                self.print_pose(marker_id, tvec, rvec)
                Xrel = tvec[0][0][0]
                Zrel = tvec[0][0][2]
                thetaArucoRel = rvec[0][0][2]
                self.calculate_robot_pos2(Xrel, Zrel, marker_id[0], thetaArucoRel)
        return frame

    def print_pose(self, marker_id, tvec, rvec):
        self.get_logger().info(f"\n=== ArUco Marker Detected ===\nMarker ID: {marker_id[0]}\nTranslation Vector (tvec):\n  X: {tvec[0][0][0]:.3f} m\n  Y: {tvec[0][0][1]:.3f} m\n  Z: {tvec[0][0][2]:.3f} m\nRotation Vector (rvec):\n  Rx: {rvec[0][0][0]:.3f} rad\n  Ry: {rvec[0][0][1]:.3f} rad\n  Rz: {rvec[0][0][2]:.3f} rad")

    def calculate_robot_pos2(self, Xrel, Zrel, aruco_id, thetaArucoRel):
        aruco_positions = self.aruco_positions
        if aruco_positions[aruco_id][0] == 0: #x minimo
            thetaArucoAbs = 0
            posXabs = Zrel
            posZabs = Xrel + aruco_positions[aruco_id][1]
        elif aruco_positions[aruco_id][0] == 7: #x maximo
            thetaArucoAbs = 180
            posXabs = aruco_positions[aruco_id][0] - Zrel
            posZabs = aruco_positions[aruco_id][1] - Xrel
        elif aruco_positions[aruco_id][1] == 0: #z minimo
            thetaArucoAbs = 90
            posXabs = aruco_positions[aruco_id][0] - Xrel
            posZabs = Zrel
        elif aruco_positions[aruco_id][1] == 7: #z maximo
            thetaArucoAbs = 270
            posXabs = aruco_positions[aruco_id][0] - Xrel
            posZabs = aruco_positions[aruco_id][1] - Zrel

        AngleRobot = thetaArucoAbs - thetaArucoRel

        if AngleRobot >= 360:
            AngleRobot = AngleRobot - 360 #encontraremos angulos mas grandes de 720

        self.get_logger().info(f"Posición del robot: X={posXabs:.3f}, Y={posZabs:.3f}, Ángulo={AngleRobot:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()