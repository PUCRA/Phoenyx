import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import subprocess
import time
import math
# import tf_transformations     # raspi
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import rclpy.duration
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Joy, PointCloud2
from std_msgs.msg import Header
from bond.msg import Status
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge      # simulacion
from sklearn.cluster import DBSCAN
import pandas as pd

class FSM_final(Node):
    def __init__(self):
        super().__init__('brain_final')
        
        #### ================= VARIABLES NODO ============= ####
        self.state = 0  # Estado inicial
        self.timer = self.create_timer(0.1, self.FSM)  # 0.1 segundos
        self.first = True
        self.tf_buffer = tf2_ros.Buffer()                
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.start_node = False
        self.simulation = True
        
        #### ================= LOCALIZACIÓN ARUCO ================= ####
        
        # Publicador para la posición resultante
        self.publisher_aruco_pos = self.create_publisher(
            Twist,
            '/aruco_pos',
            10)
        
        if self.simulation:
            # Modo simulación
            self.subscription_image = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )
            
            self.subscription_camera_info = self.create_subscription(
                CameraInfo,
                '/camera/camera_info',
                self.camera_info_callback,
                10
            )
            
        else:
            # Modo real: abrimos la cámara y cargamos calibración de ficheros
            calib_dir = os.path.expanduser('./src/phoenyx_nodes/scripts_malosh/aruco/calib_params')
            self.res = "720p"
            cam_mat_file = os.path.join(calib_dir, f'camera_matrix_{self.res}.npy')
            dist_file    = os.path.join(calib_dir, f'dist_coeffs_{self.res}.npy')
            self.camera_matrix = np.load(cam_mat_file)
            self.dist_coeffs   = np.load(dist_file)
        
        # Cargar posiciones de ArUcos desde el archivo YAML
        self.aruco_positions = self.load_aruco_positions()

        # Variables para controlar el disparo de la secuencia y almacenamiento de muestras
        self.odom_reset = False 
        self.active_sim = False
        self.measurements = []  # Lista para almacenar tuples: (posXabs, posZabs, AngleRobot)
        self.get_logger().info("ArucoDetector inicializado y listo para recibir activaciones.")

        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters_create()       # raspi
        # self.parameters = cv2.aruco.DetectorParameters()          # simulacion
        self.aruco_marker_length = 0.243  # modificable

        #### ================= PERCEPCION ================= ####

        self.number = 5 #por el culo te la inco
        


        #### ================= GUIADO ================= ####

        self.publisher_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            1
        )
        
        self.create_subscription(
            Status,
            '/bond',
            self.bond_callback,
            10
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        #variable donde guardamos waypoints
        self.waypoints = self.load_waypoints_yaml()
        
        self.odometry_recived = False 
        self.aruco_pos_state = False
        self.goal_reached = False 
        self.lidar_launched = False
        self.goal_sent = False
        self.distance = None
        self.nav2_ready = False


        #### ================= CONTROL ================= ####

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())
        
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10
        )
        
        self.scan_pub = self.create_publisher(
            LaserScan, 
            '/scan_filtered', 
            10
        )
        
        self.pub_goal = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )
        
        self.pub_points = self.create_publisher(
            PointStamped, 
            '/points', 
            10
        )

        self.joystick = self.create_subscription(
            Joy,
            '/joy',
            self.callback_mando,
            10
        )
        
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # tamaño del buffer
        )
        
        self.x = 0.0
        self.y = 0.0
        self.orientation_q = None
        self.goal_threshold = 1.0  # metros para anticipar siguiente goal
        self.timeout = 2.0
        self.goal_active = False
        self.prev_time = 0
        self.last_goal_pose = None
        self.lidar_msg = None
        self.frame_id = 'base_link'
        self.map_frame = 'map'
        self.prev_is_turning = False
        self.giro_forzado = False
        self.indice_giro_aruco = 0
        
        
        #### ================= ESCANEO DE ARUCOS ================= ####
        self.arucos = []
        self.final_poses = []
        self.final_theta = []
        self.id = None
        self.final_laps = None
        self.final_mean_counter = 0
        self.pose_aruco_final = None, None, None
        self.dist_aruco_15 = None
        # self.theta = None
        
    def FSM(self):     
        #### ================= PERCEPCION ================= ####
        if self.state == -2:
            if self.first:
                self.get_logger().info('Estado 1: ')
                self.first = False

                
            elif self.aruco_pos_state:
                self.state = 2   # Ir al estado final
                self.first = True

        


            
        #### ================= LOCALIZACION ================= ####
        
        elif self.state == 0:
            if self.first:
                self.get_logger().info('Estado 0: controlando...')
                self.first = False
            
            if self.odometry_recived and self.start_node:
                self.state = 3  # Cambia al siguiente estado 1
                self.first = True
        
        elif self.state == 1:
            if self.first:
                if not self.simulation:
                    # Inicializa VideoCapture
                    self.cap = cv2.VideoCapture("/dev/camara_color")
                    self.get_logger().info("Cámara abierta correctamente.")
                    w, h = (1280, 720) if self.res=="720p" else (640,480)
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    # Timer para leer frame a frame (ej. 30 Hz)
                    fps = 5.0
                    self.create_timer(1.0/fps, self.timer_callback)
                else:
                    self.active_sim = True

                self.first = False

            elif self.odom_reset:
                self.state = 2
                self.first = True
        
        #S0: cuando recibe odometria ejecuta callback que activa un flag indicando que puede empezar la FSM
        

        #S2: Resetea el mapa con la nueva odometria
        elif self.state == 2:
            if self.first:
                self.get_logger().info('Estado 2: Lanzando el Lidar')
                self.first = False
            if not self.lidar_launched:
                self.launch_Lidar()
                time.sleep(8)
                self.launch_planner()
                self.state = 3  # Ir al estado final
                self.first = True
        

        #### ================= WAYPOINTS ================= ####

        #S3: Esperar a nav2
        elif self.state == 3:
            if self.first:
                self.get_logger().info('Estado 3: Esperando a que NAV2 esté listo')
                self.first = False
            if self.nav2_ready:
                time.sleep(0.5)
                self.state = 4
                self.first = True

        #S4: Navegar a travès de los waypoints
        elif self.state == 4:
            if self.first:
                self.get_logger().info("Estado 4: enviando waypoints")
                self.first = False
            
            if not self.goal_reached:
                wp = self.waypoints[0]
                
                # Si no hemos enviado un goal válido todavía, lo intentamos
                if not self.goal_sent:
                    self.send_goal(wp['x'], wp['y'])
                    self.get_logger().info("Waypoint pasillo enviado.")
                    # Marcamos que hemos intentado el envío, pero no que esté aceptado
                    self.goal_sent = True
                    self.goal_reached = False
                
            else:
                self.get_logger().info("Todos los waypoints alcanzados.")
                self.state = 5
                self.goal_reached = False
                self.first = True


        #### ================= CONTROL ================= ####

        elif self.state == 5:
            if self.first:
                self.active_sim = True
                self.get_logger().info("Estado 5:⏩ Navegación continua con LiDAR iniciada")
                self.first = False

            

                   # Añadir cuando vea el aruco ID 15
            if self.lidar_msg != None: #si el mensaje del lidar existe
                if self.goal_active: 
                    distance = self.check_progress()
                    # self.get_logger().info(f"Distancia al goal: {distance:.2f} m")
                    if distance != -1 and distance < self.goal_threshold:
                        # self.get_logger().info(f"📍 Cerca del goal ({distance:.2f} m)")
                        self.goal_active = False
                    if time.time() - self.prev_time > self.timeout:
                        self.goal_active = False

            # self.get_logger().info(f"{self.arucos}")

            if self.id == 15:    
                self.state = 6
                self.first = True


        #### ================= FINAL ================= ####
        elif self.state == 6:
            if self.first:
                # self.get_logger().info("Estado 6: Parando a 5m")
                self.active_sim = True
                self.first = False
    
                
            elif self.final_mean_counter == 10:
                self.get_logger().info(f"{self.pose_aruco_final}")
                time.sleep(0.5)
                
                self.get_logger().info(f"{self.pose_aruco_final}")

                r_forward = self.dist_aruco_15 - 1.5 # cambiar a 1.5m
                # x_forward = r_forward * np.cos(self.theta)
                # y_forward = r_forward * np.sin(self.theta)
                # self.get_logger().info(f"{x_forward, y_forward}")
                self.create_and_send_goal(r_forward, 0.0, 0.0, True)
                
                # self.send_goal(self.pose_aruco_final[0],self.pose_aruco_final[1] + 5.0)
                # self.get_logger().info("Waypoint final enviado.")
                # # Marcamos que hemos intentado el envío, pero no que esté aceptado
                # self.goal_sent = True
                # self.goal_reached = False
                # goal_map_stamped = PoseStamped()
                # goal_map_stamped.header.frame_id = self.map_frame
                # goal_map_stamped.header.stamp = self.get_clock().now().to_msg()
                # goal_map_stamped.pose.position.x = float(self.pose_aruco_final[0])
                # goal_map_stamped.pose.position.y = float(self.pose_aruco_final[1] + 5.0)
                # goal_map_stamped.pose.orientation.z = math.sin(self.pose_aruco_final[2]/2.0)
                # goal_map_stamped.pose.orientation.w = math.cos(self.pose_aruco_final[2]/2.0)
                # self.get_logger().info(f"➡️ Goal final: ({goal_map_stamped.pose.position.x:.2f}, {goal_map_stamped.pose.position.y:.2f})")
                # self.pub_goal.publish(goal_map_stamped)
            
            if self.goal_reached:    
                self.state = 7
                self.first = True
        
        #S6: Estado final de reposo 
        elif self.state == 7:
            self.arucos = [point for point in self.arucos if point[2] not in (10, 11)]
            df = pd.DataFrame(self.arucos, columns=["x", "y", "ID"])

            eps_distance = 1.5  # Distancia máxima entre puntos para considerarse en el mismo grupo
            min_samples = 1     # Consideramos hasta detecciones individuales como un grupo
            
            id_especifico = self.number
            puntos = df[df['ID'] == id_especifico][['x', 'y']].to_numpy()
            
            # Verifica que hay puntos
            if len(puntos) == 0:
                print(f"No hay puntos para ID {id_especifico}")
            else:
                clustering = DBSCAN(eps=eps_distance, min_samples=min_samples).fit(puntos)
                etiquetas = clustering.labels_
                self.final_laps = len(set(etiquetas)) - (1 if -1 in etiquetas else 0)
                print(f"ID {id_especifico} tiene {self.final_laps} grupo(s)")

            self.get_logger().info(f"{self.final_laps}")
            self.get_logger().info('Estado 7: estado final alcanzado. Nada más que hacer.')
            self.timer.cancel()  # Detiene la máquina de estados

        

    #### ================= FUNCIONES PERCEPCION ================= ####
    
    def obtener_recorte(self, frame: np.ndarray log_level=0):
        try:
            if not isinstance(frame, np.ndarray):
                raise ValueError("El parámetro de entrada debe ser una imagen de OpenCV (numpy.ndarray).")
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
            if log_level == 1:
                cv2.imshow('Frame Image', frame)
            combined_mask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, combined_mask = cv2.threshold(combined_mask, 120, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            if log_level == 1:
                cv2.imshow('Combined Mask', combined_mask)

            # # Tratamiento morfológico
            combined_mask = cv2.bitwise_not(combined_mask)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
            close_img = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
            cleaned_mask = cv2.morphologyEx(close_img, cv2.MORPH_OPEN, kernel)
            # cleaned_mask = close_imgk
            # cleaned_mask = cv2.bitwise_not(cleaned_mask)
            if log_level == 1:
                cv2.imshow('Cleaned Mask', cleaned_mask)
            vertices = self.detectar_contornos(cleaned_mask)
            # print(vertices)
            if len(vertices) == 0:
                return None, cleaned_mask

            puntos_origen = np.array([vertices[2][0], vertices[3][0], vertices[0][0], vertices[1][0]], np.float32)
            puntos_origen = self.ordenar_puntos(puntos_origen)

            mask_black = np.zeros_like(frame)
            cv2.fillPoly(mask_black, [vertices], (255, 255, 255))
            result = cv2.bitwise_and(frame, mask_black)

            if log_level == 1:
                cv2.imshow('Masked Region', result)

            ancho = 800
            alto = 800
            puntos_destino = np.float32([[0, 0], [ancho, 0], [0, alto], [ancho, alto]])
            matriz = cv2.getPerspectiveTransform(puntos_origen, puntos_destino)
            numero_cuadrado = cv2.warpPerspective(result, matriz, (ancho, alto))
            frame_thickness = 20
            numero_cuadrado[-frame_thickness:, :] = 0  # Borde inferior
            numero_cuadrado[:, :frame_thickness] = 0  # Borde izquierdo
            numero_cuadrado[:frame_thickness, :] = 0  # Borde superior
            numero_cuadrado[:, -frame_thickness:] = 0  # Borde derecho
            cv2.imwrite("numero.jpg", numero_cuadrado)
            if log_level == 1:
                cv2.imshow('Corrected Image', numero_cuadrado)

            return numero_cuadrado, cleaned_mask

        except Exception as e:
            print(f"Ocurrió un error: {e}")
            print("Error en la línea:", traceback.format_exc())
            return None, None



    def publish_aruco_position(self, x, y, theta):
        msg = Twist()
        msg.linear.x = float(x-1)
        msg.linear.y = float(y-1)
        msg.angular.z = float(theta)
        self.publisher_aruco_pos.publish(msg)
        self.get_logger().info(f"Publicando posición final msg: X={msg.linear.x:.3f}, Y={msg.linear.y:.3f}, Ángulo={msg.angular.z:.3f}")
        self.odom_reset = True
    
    def load_aruco_positions(self):
        with open(os.path.expanduser('./src/final/config/posiciones_arucos.yaml'), 'r') as file:
            aruco_data = yaml.safe_load(file)
        return {aruco['id']: (aruco['position']['x'], aruco['position']['y'], aruco['orientation']) for aruco in aruco_data['arucos']}

    # Simulacion 
    def image_callback(self, msg):
        
        # if self.state == -1:
        if not self.active_sim:
                return  # No se procesa la imagen a menos que esté activado
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # Procesa la imagen para detectar ArUco y estimar la pose.
        result = self.detect_aruco_and_estimate_pose(frame)
        if self.state == -1:
            if not self.active_sim:
                return  # No se procesa la imagen a menos que esté activado
            else:
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
                        self.active_sim = False
                        self.measurements = []



        # if self.state == 5:
        #     #self.arucos = self.extract id and global pose
        #     #if self
        #     self.get_logger().error("hola")
        #     if not self.active_sim:
        #         return  # No se procesa la imagen a menos que esté activado
        #     try:
        #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #     except Exception as e:
        #         self.get_logger().error(f"Error al convertir la imagen: {e}")
        #         return

        #     # Procesa la imagen para detectar ArUco y estimar la pose.
        #     result = self.detect_aruco_and_estimate_pose(frame)
        #     self.get_logger().info(result)
            
        

    def camera_info_callback(self, msg):
        
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    # Real
    def timer_callback(self):         
        
        #si el estado es el 3 el callback de la camara gestiona reset odom
        if self.state == 3:
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
                    self.active_sim = False
                    self.measurements = []

        #si el estado es el 5 entonces la funcionalidad es distinta           
        elif self.state == 5:
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
                    self.active_sim = False
                    self.measurements = []

    def undistort_image(self, frame):
        h, w = frame.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

    def detect_aruco_and_estimate_pose(self, frame):
        frame = self.undistort_image(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, self.id, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if self.id is not None:
            for corner in corners:
                cv2.cornerSubPix(
                    gray, corner,
                    winSize=(5, 5),
                    zeroZone=(-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )

            cv2.aruco.drawDetectedMarkers(frame, corners, self.id)

            for corner, marker_id in zip(corners, self.id):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corner, self.aruco_marker_length, self.camera_matrix, self.dist_coeffs)
                
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                # self.print_pose(marker_id, tvec, rvec)


                # ✅ Convertimos el rvec a matriz de rotación y extraemos yaw desde la rotación
                R_mat, _ = cv2.Rodrigues(rvec[0][0])
                # thetaArucoRel = np.arctan2(R_mat[2, 0], R_mat[0, 0])  # Esto es yaw (ángulo del robot respecto al ArUco)
                if self.state == 1:
                    result = self.calculate_robot_pos2(tvec, R_mat, marker_id[0])
                    return result
                elif self.state == 5:
                    # self.get_logger().info(f"{self.id}")
                    result = self.transform_point_base_to_map(tvec[0][0][2], tvec[0][0][0], math.atan2(tvec[0][0][2], tvec[0][0][0]))
                    self.arucos.append([result[0], result[1], int(self.id[0][0])])
                    return result
                
                elif self.state == 6:
                    
                    self.final_mean_counter += 1 
                    # self.get_logger().info(f"{self.id}")
                    result = self.transform_point_base_to_map(tvec[0][0][2], tvec[0][0][0], math.atan2(tvec[0][0][0], tvec[0][0][2]))
                    
                    if self.final_mean_counter == 10:
                        # mean_x = np.mean(self.final_poses[0])
                        # mean_y = np.mean(self.final_poses[1])
                        # mean_yaw = np.mean(self.final_poses[2])
                        # self.pose_aruco_final = mean_x, mean_y, mean_yaw
                        # self.theta = np.mean(self.final_theta)
                        self.dist_aruco_15 = np.mean(self.final_poses)
                    else:
                        dist_aruco_15 = np.hypot(tvec[0][0][2], tvec[0][0][0])
                        self.final_poses.append(dist_aruco_15)
                        # theta = math.atan2(tvec[0][0][0], tvec[0][0][2])
                        # self.final_theta.append(theta)
                    return result
                    #calcular pos en globale

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
    
    #### ================= FUNCIONES GUIADO ================= ####     
    
    def aruco_pos_callback(self, msg):
        self.aruco_pos_state = True     # Se ha recibido un mgs por /aruco_pos
        # Extrae y almacena los datos
        self.initial_x = msg.linear.x
        self.initial_y = msg.linear.y
        self.initial_yaw = msg.angular.z  # Suponiendo que angular.z es el yaw
        self.publish_pose(0.0, 0.0, 0.0)
        self.get_logger().info(
            f"Aruco pos: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}"
        )

    def load_waypoints_yaml(self):
       """Carga los waypoints desde un archivo YAML."""
       package_name = 'final'
       package_dir = get_package_share_directory(package_name)
       yaml_path = os.path.join(package_dir, 'config', 'waypoints_final.yaml')
       try:
           with open(yaml_path ,'r') as file:
               data = yaml.safe_load(file)
               return data.get('waypoints', [])
       except Exception as e:
           self.get_logger().error(f"No se pudo cargar el YAML: {e}")
           return []

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = 0.0
        
        self.get_logger().info(f"Enviando goal de navegación: x={x}, y={y}")

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback_guiado)

    def goal_response_callback_guiado(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rechazado.')
            self.goal_sent = False
            return
        
        elif goal_handle.accepted:                                              # Se ha cmabiado esto
            self.get_logger().info('Goal aceptado, esperando resultado...')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        self.get_logger().warn("📋SE HA EJECUTADO CALLBACK RESPONSE📋")

        # result = future.result().result
        status = future.result().status
        if status == 4 or self.distance < 0.5:
            # self.goal_sent = False
            self.get_logger().info('Goal alcanzado correctamente.')
            self.goal_reached = True
            # time.sleep(1)
        elif status == 6:
            self.get_logger().info('Goal abortado.')
            self.goal_sent = False
        else:
            self.get_logger().warn(f'La navegación terminó con estado: {status}')

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: Distancia restante {feedback.distance_remaining:.2f}')
        self.distance = feedback.distance_remaining
             
    def launch_Lidar(self):
        subprocess.Popen(([
            'ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch_view.py',
        ]))
            
    def publish_pose(self, x, y, theta):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "odom"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0

            # Convert theta (yaw) to quaternion
            # q = tf_transformations.quaternion_from_euler(0, 0, theta)   # raspi
            q = self.euler_to_quaternion(0.0 ,0.0, theta)               # simulacion
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            # Covariance: usually set to low values for high confidence
            msg.pose.covariance = [0.0] * 36

            self.publisher_initialpose.publish(msg)
            self.get_logger().info(f"Published initial pose: x={x}, y={y}, theta={theta}")

    def launch_planner(self):
        subprocess.Popen(
            ['ros2', 'launch', 'planificador', 'planificador.launch.py'] # Falta añadir el yaml de guiado
        )
    
    def bond_callback(self, msg):
        if not self.nav2_ready:
            self.nav2_ready = True
            self.get_logger().info("¡bt_navigator está activo y publicando estado!")

    #### ================= FUNCIONES CONTROL ================= ####
               
    # Actualiza el mensaje del lidar
    def lidar_callback(self, msg):
        if self.state == 5:
            if (not self.goal_active) and self.start_node:
                self.lidar_msg = msg
                self.get_logger().info("Generando siguiente goal...")
                if self.id == 10 and not self.giro_forzado: #Aruco girar izquierda
                    self.indice_giro_aruco = 1 # izquierda
                    self.giro_forzado = True
                elif self.id == 11 and not self.giro_forzado: #Aruco girar derecha
                    self.indice_giro_aruco = 2 # derecha
                    self.giro_forzado = True
                elif not self.giro_forzado:
                    self.indice_giro_aruco = 0
                x_forward, y_lateral, yaw = self.generate_goal_from_lidar(self.lidar_msg, self.indice_giro_aruco)
                if x_forward is None:
                    return
                goal, error = self.create_and_send_goal(x_forward, y_lateral, yaw)
                self.goal_active = not error
                # if self.goal_active:
                #     time.sleep(5.0)
                self.prev_time = time.time()
                if error:
                    self.get_logger().warning("Error al generar el goal")
                else:
                    self.last_goal_pose = goal
            
    # Checkea cuanta distancia queda para llegar al goal
    def check_progress(self):
        if not self.goal_active or self.last_goal_pose is None:
            return -1
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            goal_x = self.last_goal_pose.pose.position.x
            goal_y = self.last_goal_pose.pose.position.y

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            return distance
        
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"[TF Error] al verificar progreso: {e}")
            return -1

    def generate_goal_from_lidar(self, msg, giro = 0):
        # msg = self.rotate_laserscan(msg, np.radians(-90))
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Usamos un rango frontal de -80° a 80° (más sensible a giros)
        mask = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(80))
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        if len(valid_ranges) == 0:
            self.get_logger().warn("🚧 No hay datos válidos en -80° a 80°.")
            return

        # Suavizamos para quitar ruido
        smooth_ranges = np.convolve(valid_ranges, np.ones(3)/3, mode='same')
        new_ranges = []
        new_angles = []

        # Filtramos puntos cercanos
        for i in range(0, len(smooth_ranges)):
            # Filtramos puntos muy cercanos a la pared (por ejemplo, < 1.0 metros)
            if smooth_ranges[i] > 1.0:
                new_ranges.append(smooth_ranges[i])
                new_angles.append(valid_angles[i])

        if len(new_ranges) == 0:
            self.get_logger().warn("🚧 No hay puntos válidos después del filtrado.")
            return

        # Promediamos los puntos LIDAR en bloques de 10
        valid_ranges_2, valid_angles_2 = self.average_lidar_in_blocks(new_ranges, new_angles, block_size=10)

        # Calculamos las posiciones en x e y de los puntos promediados
        x_points = valid_ranges_2 * np.cos(valid_angles_2)
        y_points = valid_ranges_2 * np.sin(valid_angles_2)

        # Calculamos la media de las posiciones x e y
        goal_x = np.mean(x_points)
        goal_y = np.mean(y_points)

        mask = (np.isfinite(ranges)) & (np.radians(-5) <= angles) & (angles <= np.radians(5))
        front_distance = ranges[mask]
        front_distance = max(front_distance)
        if front_distance < 2.2 or self.giro_forzado:
            self.get_logger().warning(f"Pared detectada: {front_distance:.2f} m")
            angle = np.radians(0)
            sentido = 1
            if giro == 0: # Lidar
                mask_left = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(-10))
                mask_right = (np.isfinite(ranges)) & (np.radians(10) <= angles) & (angles <= np.radians(80))
                left_distance = ranges[mask_left]
                right_distance = ranges[mask_right]
                max_left = max(left_distance)
                max_right = max(right_distance)
                if max_left > max_right:
                    sentido = -1
                else:
                    sentido = 1
                self.giro_forzado = False
            elif giro == 1: # Izquierda si o si
                sentido = -1
                self.giro_forzado = False
                self.get_logger().info("=========================FORZANDO GIRO IZQUIERAD ================================")
            elif giro == 2: # Derecha si o si
                sentido = 1
                self.giro_forzado = False
                self.get_logger().info("=========================FORZANDO GIRO DERECHA ================================")
            angle = sentido*np.radians(45)
            if front_distance < 1.0:
                front_distance = 1.0
                angle += sentido*np.radians(15)
            if self.prev_is_turning:
                angle = abs(angle)/angle*np.radians(30)
            if abs(front_distance) < 1.5:
                front_distance = 1.5
            front_distance = abs((front_distance-0.75)/np.cos(angle))
            goal_x = front_distance*np.cos(angle)
            goal_y = front_distance*np.sin(angle)
            
        else:
            self.giro_forzado = False
            for i in range(len(x_points)):
                error_x = goal_x - x_points[i]
                error_y = goal_y - y_points[i]
                error = math.sqrt(error_x**2 + error_y**2)
                if error < 0.6:
                    self.get_logger().info(f"🚧 Punto cercano a la pared: ({x_points[i]:.2f}, {y_points[i]:.2f}), error: {error:.2f} m")
                    goal_x *= 0.5
                    goal_y *= 0.5
                    # goal_x += (goal_x - x_points[i]) * 0.5  # Ajuste proporcional
                    # goal_y += (goal_y - y_points[i]) * 0.5
                    break

        # Orientamos el goal hacia el ángulo calculado
        best_angle = math.atan2(goal_y, goal_x)
        if giro > 0:
            self.timeout = 10.0
        if abs(best_angle) >= 15.0:
            self.prev_is_turning = True
            self.timemout = 4.0
        else:
            self.prev_is_turning = False
            self.timeout = 1.5
        # Aseguramos que el ángulo esté en un rango razonable
        if abs(best_angle) > math.radians(90):  # Evitar giros excesivos
            self.get_logger().warn(f"⚠️ Ángulo excesivo de giro ({math.degrees(best_angle):.1f}°). Ajustando...")
            best_angle = np.sign(best_angle) * math.radians(90)

        self.get_logger().info(f"🎯 Nuevo goal: ({goal_x:.2f}, {goal_y:.2f}), yaw: {math.degrees(best_angle):.1f}°")
        return goal_x, goal_y, best_angle

    def rotate_laserscan(self, scan_msg: LaserScan, angle_shift_rad: float) -> LaserScan:
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        shift = int(angle_shift_rad / angle_increment)
        # self.get_logger().info("Rotating...")
        # Rota el array circularmente
        rotated_ranges = np.roll(ranges, shift)

        # Crear nuevo mensaje corregido
        corrected_scan = LaserScan()
        corrected_scan.header = scan_msg.header
        corrected_scan.header.frame_id = 'base_link'  # o lo que quieras
        corrected_scan.angle_min = scan_msg.angle_min
        corrected_scan.angle_max = scan_msg.angle_max
        corrected_scan.angle_increment = scan_msg.angle_increment
        corrected_scan.time_increment = scan_msg.time_increment
        corrected_scan.scan_time = scan_msg.scan_time
        corrected_scan.range_min = scan_msg.range_min
        corrected_scan.range_max = scan_msg.range_max
        corrected_scan.ranges = rotated_ranges.tolist()
        corrected_scan.intensities = scan_msg.intensities
        # self.get_logger().info("Rotated!")

        return corrected_scan

    def average_lidar_in_blocks(self, ranges, angles, block_size=10):
        # Nos aseguramos de que sean arrays de numpy
        ranges = np.array(ranges)
        angles = np.array(angles)

        # Cortamos para que encaje bien en bloques
        n = len(ranges) - (len(ranges) % block_size)
        ranges = ranges[:n]
        angles = angles[:n]

        # Reshape para hacer bloques
        range_blocks = ranges.reshape(-1, block_size)
        angle_blocks = angles.reshape(-1, block_size)

        # Calculamos promedios por bloque
        avg_ranges = np.mean(range_blocks, axis=1)
        avg_angles = np.mean(angle_blocks, axis=1)

        return avg_ranges, avg_angles

    # Transforma el goal de odom a map y prepara el mensaje para nav2
    def create_and_send_goal(self, x_forward, y_lateral, yaw, transform = True):
        try:
            if transform:
                x, y, yaw = self.transform_point_base_to_map(x_forward, y_lateral, yaw)
            else:
                x = x_forward
                y = y_lateral
                yaw = yaw
            if x is None or y is None or yaw is None:
                return None, 1
            else:
                goal_map_stamped = PoseStamped()
                goal_map_stamped.header.frame_id = self.map_frame
                goal_map_stamped.header.stamp = self.get_clock().now().to_msg()
                goal_map_stamped.pose.position.x = float(x)
                goal_map_stamped.pose.position.y = float(y)
                goal_map_stamped.pose.orientation.z = math.sin(yaw/2.0)
                goal_map_stamped.pose.orientation.w = math.cos(yaw/2.0)
                self.get_logger().info(f"➡️ Nuevo goal dinámico: ({goal_map_stamped.pose.position.x:.2f}, {goal_map_stamped.pose.position.y:.2f})")
                self.pub_goal.publish(goal_map_stamped)
                return goal_map_stamped, 0
                

        except Exception as e:
            self.get_logger().warn(f"Error al transformar goal: {e}")
            return None, 1

    # Devuelve si el goal ha sido aceptado o rechazado
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rechazado por Nav2.")
            self.goal_active = False
            return

        self.get_logger().info("✅ Goal aceptado")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # Detecta el resultado del goal
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().warn("🚫 Goal abortado.")
            self.goal_active = False
        elif status == 3:
            self.get_logger().info("🎯 Goal alcanzado (aunque ya se generó otro).")

        self.goal_active = False

    def get_yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw  # En radianes

    def transform_point_base_to_map(self, x_base, y_base, yaw_base):
        point_stamped = PoseStamped()
        point_stamped.header.stamp = Time(sec=0, nanosec=0)  # Se puede usar el tiempo actual
        point_stamped.header.frame_id = self.frame_id
        point_stamped.pose.position.x = float(x_base)
        point_stamped.pose.position.y = float(y_base)
        point_stamped.pose.position.z = 0.0
        point_stamped.pose.orientation.z = math.sin(yaw_base/2.0)
        point_stamped.pose.orientation.w = math.cos(yaw_base/2.0)
        # Transformamos el punto al frame map
        try:
            target_frame = 'map'
            transformed_point = self.tf_buffer.transform(
                point_stamped,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # self.get_logger().info("Tranformando")
            yaw = self.get_yaw_from_quaternion(transformed_point.pose.orientation.x,
                                               transformed_point.pose.orientation.y,
                                               transformed_point.pose.orientation.z,
                                               transformed_point.pose.orientation.w)
            return transformed_point.pose.position.x, transformed_point.pose.position.y, yaw
        except Exception as e:
            self.get_logger().warn(f"No se pudo transformar: {e}")
            return None, None, None
        
    #### ================= FUNCIONES FUSIONADAS ================= ####

    def odom_callback(self, msg):    
        self.odometry_recived = True    # Se ha recibido un msg por /odom
        if self.state == 5:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.orientation_q = msg.pose.pose.orientation

    # Transforma de euler a quaternion
    def euler_to_quaternion(self, roll, pitch, yaw):

        # Convertimos Euler (roll, pitch, yaw) a cuaternión
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        return [qx, qy, qz, qw]
    
    def callback_mando(self, msg):
        if (not self.start_node) and msg.buttons[0]:
            self.get_logger().info("Iniciando nodo")
            self.start_node = msg.buttons[0] # Boton A
    ### PREPARACION DE FUNCIONES PARA EL ESTADO ULTIMO ##
    def calculate_laps(self, arucos):
        "Hace un clustering de la variable self.arucos(lista ya hecha) y actualiza self.final_laps con el numero de vueltas al final de la prueba"
        arucos = [point for point in self.arucos if point[2] not in (10, 11)] #filtramos para sacar id 11 y 12 
        df = pd.DataFrame(arucos, columns=["x", "y", "ID"])#DBSCAN(tecnica de clustering) funciona con df

        eps_distance = 1.5  # Distancia maxima entre puntos para considerarse en el mismo grupo
        min_samples = 1     # Consideramos hasta detecciones individuales como un grupo
        
        id_especifico = self.number #de todo el array cogemos los elementos que contengan la id del numero detectado
        puntos = df[df['ID'] == id_especifico][['x', 'y']].to_numpy()
        
        # Verifica que hay puntos
        if len(puntos) == 0:
            print(f"No hay puntos para ID {id_especifico}")
        else:
            clustering = DBSCAN(eps=eps_distance, min_samples=min_samples).fit(puntos)
            etiquetas = clustering.labels_
            final_laps = len(set(etiquetas)) - (1 if -1 in etiquetas else 0) 
            # aqui no se si es mejor simplemente actualizar self.final_laps o hacer un return del numero de vueltas
            # si asi se desea, descomentar:
            return final_laps
            # print(f"ID {id_especifico} tiene {self.final_laps} grupo(s)/vueltas") 
        

def main(args=None):
    rclpy.init(args=args)
    node = FSM_final()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
