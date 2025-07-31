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
import tf_transformations     # raspi
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
# from cv_bridge import CvBridge      # simulacion
from sklearn.cluster import DBSCAN
import pandas as pd
from final.Recorte2number import Recorte2number
from std_msgs.msg import Int32

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
        self.simulation = False

        #### ================= LOCALIZACIÓN ARUCO ================= ####
        # Variable para almacenar el frame de la cámara
        self.simulation = False
        # self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_marker_length = 0.285  # modificable
        self.frame = None
        # Publicador para la posición resultante
        self.publisher_aruco_pos = self.create_publisher(
            Twist,
            '/aruco_pos',
            10)
        # Modo real: abrimos la cámara y cargamos calibración de ficheros
        self.pub_vueltas = self.create_publisher(Int32, '/num_vueltas', 10)
        calib_dir = os.path.expanduser('./src/phoenyx_nodes/scripts_malosh/aruco/calib_params')
        res = "480p"
        cam_mat_file = os.path.join(calib_dir, f'camera_matrix_{res}.npy')
        dist_file    = os.path.join(calib_dir, f'dist_coeffs_{res}.npy')
        self.camera_matrix = np.load(cam_mat_file)
        self.dist_coeffs   = np.load(dist_file)
        # Inicializa VideoCapture
        self.cap = cv2.VideoCapture("/dev/camara_color")
        self.get_logger().info("Cámara abierta correctamente.")
        w, h = (1280, 720) if res=="720p" else (640,480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        # Timer para leer frame a frame (ej. 30 Hz)
        fps = 0.5
        self.create_timer(1.0/fps, self.image_callback)
        self.get_logger().info("Timer creado para lectura de cámara.")
        # Cargar posiciones de ArUcos desde el archivo YAML
        self.aruco_positions = self.load_aruco_positions()

        # Variables para controlar el disparo de la secuencia y almacenamiento de muestras
        self.odom_reset = False
        self.measurements = []  # Lista para almacenar tuples: (posXabs, posZabs, AngleRobot)
        self.get_logger().info("ArucoDetector inicializado y listo para recibir activaciones.")

        #### ================= PERCEPCION ================= ####

        #Detección del color y numero
        #Dar una vuelta al detectarlos en el sentido que queramos para que retiren caja
        self.number = 5   # Para la contabilizacion de arucos de utiliza self.numbers, que viene de percepcion. Hay que juntar las dos partes del codigo
        self.numero_muestras = 10
        self.numeros = []
        self.colores = []
        self.conteo_muestras = 0
        self.conversor = Recorte2number()
        self.pub_vueltas = self.create_publisher(Int32, '/num_vueltas', 10)
        #### ================= GUIADO ================= ####
        self.odom_published = False
        self.create_subscription(
            Status,
            '/bond',
            self.bond_callback,
            10
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

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
        self.timeout = 1.8
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
        self.final_laps = None
        self.final_mean_counter = 0
        self.pose_aruco_final = None, None, None
        self.dist_aruco_15 = None

    def FSM(self):
        # ========== Percepción ========== #
        if self.state == 0: # Esperar activación con el mando y odometria (Hace falta esoerar a odom? o mejor para girar (estado 3))
            if self.first:
                self.get_logger().info('Estado 0: Todo listo para iniciar, pulsa A')
                self.first = False

            if self.start_node:
                self.state = 1
                self.first = True

        elif self.state == 1: # Detectar Caja con numero
            if self.first:
                self.get_logger().info('Estado 1: Detectando caja')
                self.first = False
            # ret, frame = self.cap.read()
            # if not ret:
            #     return
            # recorte, _ = self.obtener_recorte(frame)
            # if recorte is not None:
            #     progreso = len(self.numeros) / self.numero_muestras
            #     porcentaje = int(progreso * 100)
            #     barra = "#" * (porcentaje // 2)  # Barra de 50 caracteres máx.
            #     espacio = " " * (50 - len(barra))  # Relleno para mantener tamaño fijo
            #     self.get_logger().info(f"[{barra}{espacio}] {porcentaje}%")
            #     self.tratar_recorte(recorte)
            time.sleep(10)
            if True:
                self.state = 2
                self.first = True

        elif self.state == 2: # Decision making
            if self.first:
                self.get_logger().info('Estado 2: Computando decision de color y numero')
                self.first = False
            # self.number, self.color_final = self.decision_making()
            # self.get_logger().info("Numeros: {}".format(self.numeros))
            # numero_print = str(self.number)
            # if self.number == 0:
            #     numero_print = "No hay numero"
            # self.get_logger().info("Numero: "+numero_print+" Color: "+str(self.color_final))
            msg = Int32()
            msg.data = 1
            self.pub_vueltas.publish(msg)

            self.state = 3
            self.first = True

        elif self.state == 3: # Girar
            if self.first:
                self.get_logger().info('Estado 3: Girando una vuelta...')
                self.first = False
            # msg.data = 1
            # self.pub_vueltas.publish(msg)
            time.sleep(30)
            if True:
                self.state = 4
                self.first = True

        # ========== Localización ========== #
        elif self.state == 4: # Localizar ArUco y enviar posición
            if self.first:
                self.get_logger().info('Estado 4: Localizando Robot con ArUco')
                self.first = False
            if self.frame is not None:
                marker_id, rvec, tvec, result = self.detect_aruco_and_estimate_pose(self.frame, True)
                if result is not None:
                    self.measurements.append(result)
                    self.get_logger().info(f"Medición {len(self.measurements)}/30 obtenida.")
                    if len(self.measurements) >= 5:
                        # Aplica filtro mediano a cada uno de los valores
                        posX_list, posZ_list, angle_list = zip(*self.measurements)
                        posX_med = np.median(posX_list)
                        posZ_med = np.median(posZ_list)
                        angle_med = np.median(angle_list)
                        # Publica el resultado único
                        self.publish_aruco_position(posX_med, posZ_med, angle_med)
                        self.odom_published = True
                        # Reinicia el proceso para futuras activaciones
                        self.active_sim = False
                        self.measurements = []

            if self.odometry_recived and self.odom_published:
                self.launch_lidar()
                time.sleep(5)
                self.launch_planner()
                self.state = 5

        elif self.state == 5: # esperar a nav2
            if self.first:
                self.get_logger().info('Estado 5: Wait a nav2')
                self.first = False

            if self.nav2_ready:
                time.sleep(5)
                self.state = 6
                self.first = True

        elif self.state == 6: # Enviar el primer waypoint hacia el pasillo
            if self.first:
                self.get_logger().info('Estado 6: Enviando el primer waypoint')
                self.first = False

            if not self.goal_reached:
                # Si no hemos enviado un goal válido todavía, lo intentamos
                if not self.goal_sent:
                    self.send_goal(1.5, 5.0) #Mandamos un waypoint unico y exclusivo del pasillo
                    self.get_logger().info("Waypoint pasillo enviado.")
                    # Marcamos que hemos intentado el envío, pero no que esté aceptado
                    self.goal_sent = True
                    self.goal_reached = False

            else:
                self.get_logger().info("Waypoint alcanzado")
                # self.image_callback.cancel()
                self.state = 7
                self.goal_reached = False
                self.first = True

        # ========== Control ========== #

        elif self.state == 7: #
            if self.first:
                self.get_logger().info('Estado 7: Navegación continua con LiDAR iniciado')
                self.first = False

            if self.lidar_msg != None: #si el mensaje del lidar existe
                if self.goal_active:
                    distance = self.check_progress()
                    # self.get_logger().info(f"Distancia al goal: {distance:.2f} m")
                    if (distance != -1 and distance < self.goal_threshold) or (time.time() - self.prev_time > self.timeout):
                        # self.get_logger().info(f"📍 Cerca del goal ({distance:.2f} m)")
                        self.goal_active = False
                else:
                    doing_lidar = True
                    ret, self.frame = self.cap.read()
                    if self.frame is not None:
                        id_aruco, rvec, tvec, robto_poser = self.detect_aruco_and_estimate_pose(self.frame)
                        if id_aruco != 10 and id_aurco != 11 and rvec is not None and tvec is not None:
                            result = self.transform_point_base_to_map(tvec[0][0][2], tvec[0][0][0],
                                                                 math.atan2(tvec[0][0][0], tvec[0][0][2]))
                            self.arucos.append([result[0], result[1], int(id_aruco)])
                        if self.giro_forzado:
                            self.frame = None
                            pass
                        elif id_aruco == 15:
                            x = tvec[0][0][2]-5.0
                            if x < 0.5:
                                x = 0.5
                            goal, error = self.send_goal(x, 0.0, 0.0, True, False)
                            if error:
                                self.get_logger().warning("Error al generar el goal")
                            else:
                                self.last_goal_pose = goal
                                self.state = 8
                                self.first = True
                            doing_lidar = False
                        elif id_aruco == 11: #Aruco girar izquierda
                            self.frame = None
                            indice_giro_aruco = 1 # izquierda
                            self.giro_forzado = True
                        elif id_aruco == 10: #Aruco girar izquierda
                            self.frame = None
                            indice_giro_aruco = 2 # izquierda
                            self.giro_forzado = True
                        else:
                            indice_giro_aruco = 0
                            self.frame = None

                    else:
                        indice_giro_aruco = 0
                    if doing_lidar:
                        x_forward, y_lateral, yaw = self.generate_goal_from_lidar(self.lidar_msg, indice_giro_aruco)
                        if x_forward is None:
                            return

                        goal, error = self.send_goal(x_forward, y_lateral, yaw, True, False)
                        self.goal_active = not error
                        self.prev_time = time.time()
                        if error:
                            self.get_logger().warning("Error al generar el goal")
                        else:
                            self.last_goal_pose = goal

        elif self.state == 8: # Esperar a que el robot pare
            if self.first:
                self.get_logger().info('Estado 8: Mandando ultimo waypoint')
                self.first = False
                self.goal_active = False
                time.sleep(10)
            if len(self.final_poses) < 5:
                self.goal_active = False
                ret, frame = self.cap.read()
                id_aruco, rvec, tvec, robot_pose = self.detect_aruco_and_estimate_pose(frame)
                if id_aruco == 15:
                    self.final_poses.append((rvec, tvec))
                    self.get_logger().info(f"Longitud: {len(self.final_poses)}")
                    self.get_logger().info(f"Posición ArUco 15 guardada: {self.final_poses[-1]}")
            elif not self.goal_active:
                x, y, yaw = self.compute_goal_in_robot_frame(self.final_poses, 1.5)
                self.get_logger().info(f"Ultimo waypoint: x={x}, y={y}, yaw={yaw}")
                goal, error = self.send_goal(x, y, yaw, True, False)
                self.goal_active = not error
            else:
                distance = self.check_progress()
                if distance != -1 and distance < 0.5:
                    self.get_logger().info(f"📍 Cerca del goal ({distance:.2f} m)")
                    self.state = 9
                    self.first = True

        elif self.state == 9: # Dar N vueltas como arucos detectados
            if self.first:
                vueltas = self.calculate_laps(self.arucos)
                # self.get_logger().info(f"{self.arucos}")
                msg = Int32()
                if vueltas < 2:
                    vueltas = 2
                msg.data = vueltas
                self.pub_vueltas.publish(msg)
                self.get_logger().info(f"Estado 9: Dando {vueltas} vueltas")
                self.first = False


            # if True:
            #     self.state = 10
            #     self.first = True

    # ================== Funciones Percepcion ================== #

    def decision_making(self):
        numeros = self.numeros
        colores = self.colores
        prob_rojo = 0  # Probabilidad de rojo
        prob_azul = 0 # Probabilidad de azul
        prob_distract = 0
        for color in colores:
            if color == "Azul":
                prob_azul += 1
            elif color =="Rojo":
                prob_rojo += 1
            else:
                prob_distract += 1
        prob_rojo /= float(len(colores))
        prob_azul /= float(len(colores))
        prob_distract /= float(len(colores))
        frecuencia_por_numero = {i: 0 for i in range(0, 10)}

        for valor in numeros:
            # Filtrar por confianza: solo contar si la confianza supera el umbral
            if valor in frecuencia_por_numero:
                frecuencia_por_numero[valor] += 1  # Ponderar por la confianza

        # Determinar el número con mayor frecuencia ponderada
        numero = max(frecuencia_por_numero, key=frecuencia_por_numero.get)
        color = max(prob_rojo, prob_azul, prob_distract)
        # prob_numero = frecuencia_por_numero[numero] / sum(frecuencia_por_numero.values())  # Frecuencia relativa ponderada
        print(self.colores)
        # Determinar el color con mayor probabilidad
        if prob_rojo == color:
            color = "Rojo"
            prob_color = prob_rojo
        elif prob_azul == color:
            color = "Azul"
            prob_color = prob_azul
        else:
            color = "Distractorio"

        return numero, color

    def tratar_recorte(self, image):
        # imagen = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        numero, color, img = self.conversor.obtener_colorYnum(image)
        # self.publisher_recorte_bin_2.publish(self.bridge.cv2_to_imgmsg(image_thresh, encoding='mono8'))
        if numero is not None:
            self.numeros.append(numero)
            self.get_logger().info("Numero: "+str(numero))
        if color is not None:
            self.colores.append(color)
        self.conteo_muestras += 1
        # self.get_logger().info('Color: '+color+'Numero: '+str(numero))

    def ordenar_puntos(self, pts):
        # Convertir a un array 2D normal si es necesario
        pts = pts.reshape(4, 2)
        # Ordenar por suma (x + y) para identificar esquinas
        suma = pts.sum(axis=1)
        diferencia = np.diff(pts, axis=1)
        # Superior izquierdo: menor suma
        punto_sup_izq = pts[np.argmin(suma)]
        # Inferior derecho: mayor suma
        punto_inf_der = pts[np.argmax(suma)]
        # Superior derecho: menor diferencia (x - y)
        punto_sup_der = pts[np.argmin(diferencia)]
        # Inferior izquierdo: mayor diferencia (x - y)
        punto_inf_izq = pts[np.argmax(diferencia)]
        return np.array([punto_sup_izq, punto_sup_der, punto_inf_izq, punto_inf_der], dtype=np.float32)
    
        
    def detectar_contornos(self, frame):
        vertices = []
        contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 4:
                    area = cv2.contourArea(approx)
                    # print(area)
                    if area < 40000 and area > 5000:
                        if len(approx) == 4:
                            x, y, w, h = cv2.boundingRect(approx)
                            aspect_ratio = float(w) / h  # Relación de aspecto
                            # Verificar que la relación de aspecto sea aproximadamente 1
                            if 0.9 <= aspect_ratio <= 1.1:
                                # Verificar que los ángulos sean cercanos a 90 grados
                                angles = []
                                for i in range(4):
                                    p1 = approx[i][0]
                                    p2 = approx[(i + 1) % 4][0]
                                    p3 = approx[(i + 2) % 4][0]
                                    v1 = p1 - p2
                                    v2 = p3 - p2
                                    cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                                    angle = np.arccos(cosine_angle) * (180.0 / np.pi)
                                    angles.append(angle)
                                # Comprobar si los ángulos están cerca de 90 grados
                                if all(80 <= ang <= 100 for ang in angles):
                                    # cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                                    # print("🟢 Se ha detectado un cu
                                    return approx                    
        return vertices

    def obtener_recorte(self, frame: np.ndarray, log_level=0):
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

    def image_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error("Error al leer el frame de la cámara.")
            return
        self.frame : np.ndarray= self.undistort_image(frame)

    def undistort_image(self, frame):
        h, w = frame.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

    # ================= Funciones Localicacion ================== #
    def publish_aruco_position(self, x, y, theta):
        msg = Twist()
        msg.linear.x = float(x-1)
        msg.linear.y = float(y-1)
        msg.angular.z = float(theta)
        self.publisher_aruco_pos.publish(msg)
        self.get_logger().info(f"Publicando posición final msg: X={msg.linear.x:.3f}, Y={msg.linear.y:.3f}, Ángulo={msg.angular.z:.3f}")

    def load_aruco_positions(self):
        with open(os.path.expanduser('./src/final/config/posiciones_arucos.yaml'), 'r') as file:
            aruco_data = yaml.safe_load(file)
        return {aruco['id']: (aruco['position']['x'], aruco['position']['y'], aruco['orientation']) for aruco in aruco_data['arucos']}

    def detect_aruco_and_estimate_pose(self, frame, robotPose = False):
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
                if robotPose:
                    # ✅ Convertimos el rvec a matriz de rotación y extraemos yaw desde la rotación
                    R_mat, _ = cv2.Rodrigues(rvec[0][0])
                    # thetaArucoRel = np.arctan2(R_mat[2, 0], R_mat[0, 0])  # Esto es yaw (ángulo del robot respecto al ArUco)
                    result = self.calculate_robot_pos2(tvec, R_mat, marker_id[0])
                else:
                    result = None
                return marker_id[0], rvec, tvec, result

        return None, None, None, None

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
        self.get_logger().info(
            f"Aruco pos: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}"
        )

    def launch_lidar(self):
        subprocess.Popen(([
            'ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch_view.py',
        ]))

    def launch_planner(self):
        subprocess.Popen(
            ['ros2', 'launch', 'final', 'planificador.launch.py'] # Falta añadir el yaml de guiado
        )


    # ================== CONTROL ================== #
    def lidar_callback(self, msg):
        self.lidar_msg = msg

    def generate_goal_from_lidar(self, msg, giro = 0):
        msg = self.rotate_laserscan(msg, np.radians(-90))
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Usamos un rango frontal de -80° a 80° (más sensible a giros)
        mask = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(80))
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        if len(valid_ranges) == 0:
            self.get_logger().warn("🚧 No hay datos válidos en -80° a 80°.")
            return None, None, None

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
            return None, None, None

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
        if front_distance < 2.2:
            self.get_logger().warning(f"Pared detectada: {front_distance:.2f} m")
            angle = np.radians(0)
            sentido = 0
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
                # self.giro_forzado = False
                self.get_logger().warning("=========================FORZANDO GIRO DERECHA ================================")
            elif giro == 2: # Derecha si o si
                sentido = 1
                # self.giro_forzado = False
                self.get_logger().warning("=========================FORZANDO GIRO IZQUIERDA ================================")
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
            # self.giro_forzado = False
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
            self.timeout = 5.0
            self.giro_forzado = False
        elif abs(best_angle) >= 15.0:
            self.prev_is_turning = True
            self.timemout = 4.0
        else:
            self.prev_is_turning = False
            self.timeout = 1.8
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

    # ================== NAVEGACION ================== #

    def compute_goal_in_robot_frame(self,pose_list, distance_ahead=1.5):
        """
        Calcula el punto objetivo (x, y, yaw) en el frame del robot, a 'distance_ahead' metros
        frente al marcador ArUco.

        Args:
            pose_list: lista de (rvec, tvec)
            distance_ahead: distancia frente al marcador (en -Z del ArUco)
            T_cam_to_robot: matriz 4x4 (homogénea) de transformación de cámara a robot

        Returns:
            x_robot, y_robot, yaw_robot
        """
        import numpy as np
        import cv2

        # if T_cam_to_robot is None:
        #     raise ValueError("Falta T_cam_to_robot (matriz 4x4 de cámara a robot)")

        # 1. Separar rvecs y tvecs
        rvecs = np.array([rvec.flatten() for rvec, _ in pose_list])
        tvecs = np.array([tvec.flatten() for _, tvec in pose_list])

        # 2. Medianas
        rvec_med = np.median(rvecs, axis=0).reshape(3, 1)
        tvec_med = np.median(tvecs, axis=0).reshape(3, 1)

        # 3. Convertir rvec a matriz de rotación
        R_med, _ = cv2.Rodrigues(rvec_med)

        # 4. Punto a 1.5m frente al marcador (en su -Z)
        p_aruco = np.array([[0], [0], [-distance_ahead]])

        # 5. Transformar al marco de la cámara
        p_cam = R_med @ p_aruco + tvec_med  # 3x1

        # 6. Convertir a vector homogéneo (4x1)
        p_cam_h = np.vstack((p_cam, [[1]]))  # shape (4,1)

        T_cam_to_robot = np.array([
            [0, 0, 1, 0],   # X_robot ← Z_cam
            [1, 0, 0, 0],   # Y_robot ← X_cam
            [0, -1, 0, 0],  # Z_robot ← -Y_cam
            [0, 0, 0, 1]
        ])
        # 7. Transformar al marco del robot
        p_robot_h = T_cam_to_robot @ p_cam_h
        x_robot, y_robot = p_robot_h[0, 0], p_robot_h[1, 0]

        # 8. Calcular yaw hacia el ArUco en el frame del robot
        # También transformar el ArUco al frame del robot
        tvec_med_h = np.vstack((tvec_med, [[1]]))

        aruco_in_robot = T_cam_to_robot @ tvec_med_h
        self.get_logger().info(f"ArUco en robot: {aruco_in_robot}")
        dx = aruco_in_robot[0, 0] - x_robot
        dy = aruco_in_robot[1, 0] - y_robot
        yaw_robot = np.arctan2(dy, dx)

        return x_robot, y_robot, yaw_robot



    def odom_callback(self, msg):
        if not self.odometry_recived:
            self.odometry_recived = True

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

    def send_goal(self, x, y, yaw = 0.0, transform = False, actionServer = True):
        try:
            if transform:
                x, y, yaw = self.transform_point_base_to_map(x, y, yaw)
            goal_map_stamped = PoseStamped()
            goal_map_stamped.header.frame_id = self.map_frame
            goal_map_stamped.header.stamp = self.get_clock().now().to_msg()
            goal_map_stamped.pose.position.x = float(x)
            goal_map_stamped.pose.position.y = float(y)
            goal_map_stamped.pose.orientation.z = math.sin(yaw/2.0)
            goal_map_stamped.pose.orientation.w = math.cos(yaw/2.0)
            self.get_logger().info(f"Enviando goal de navegación: x={x}, y={y}")
            if actionServer:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = goal_map_stamped
                self._send_goal_future = self.nav_to_pose_client.send_goal_async(
                    goal_msg
                )
                self._send_goal_future.add_done_callback(self.goal_response_callback_guiado)
            else:

                self.pub_goal.publish(goal_map_stamped)
            return goal_map_stamped, 0
        except Exception as e:
            self.get_logger().warn(f"Error al transformar goal: {e}")
            return None, 1


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
        if status == 4:
            # self.goal_sent = False
            self.get_logger().info('Goal alcanzado correctamente.')
            self.goal_reached = True
            # time.sleep(1)
        elif self.distance is not None and self.distance < 0.5:
            self.get_logger().info('Goal alcanzado correctamente.')
            self.goal_reached = True
        elif status == 6:
            self.get_logger().info('Goal abortado.')
            self.goal_sent = False
        else:
            self.get_logger().warn(f'La navegación terminó con estado: {status}')

    def bond_callback(self, msg):
        if not self.nav2_ready:
            self.nav2_ready = True
            self.get_logger().info("¡bt_navigator está activo y publicando estado!")

    def callback_mando(self, msg):
        if (not self.start_node) and msg.buttons[0]:
            self.get_logger().info("Iniciando nodo")
            self.start_node = msg.buttons[0] # Boton A

    # ================== Transformaciones ================== #
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

    # Transforma de euler a quaternion
    def euler_to_quaternion(self, roll, pitch, yaw):

        # Convertimos Euler (roll, pitch, yaw) a cuaternión
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def calculate_laps(self, arucos):
        "Hace un clustering de la variable self.arucos(lista ya hecha) y actualiza self.final_laps con el numero de vueltas al final de la prueba"
        arucos = [point for point in self.arucos if point[2] not in (10, 11)] #filtramos para sacar id 10 y 11
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
    try:
        rclpy.init(args=args)
        node = FSM_final()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()