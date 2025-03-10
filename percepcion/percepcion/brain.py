import sklearn
from percepcion.Recorte2number import Recorte2number
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int32
import message_filters
from message_filters import ApproximateTimeSynchronizer
from percepcion.Img2recorte import image2recorte
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState, Joy 
import time
import numpy as np

class brain_percepcion(Node):
    def __init__(self):
        super().__init__('brain_percepcion')

        # paramns camara
        self.declare_parameters(namespace='', parameters=[
            ('depth_filter.min', 1000), 
            ('depth_filter.max', 2000),
            ('numero_muestras', 10)
        ])
        # Leer los parámetros
        self.min_depth = self.get_parameter('depth_filter.min').get_parameter_value().integer_value
        self.max_depth = self.get_parameter('depth_filter.max').get_parameter_value().integer_value
        self.numero_muestras = self.get_parameter('numero_muestras').get_parameter_value().integer_value
        self.get_logger().info('Rango de profundidad: %d - %d' % (self.min_depth, self.max_depth))
        self.get_logger().info('Numero muestras: %d' % (self.numero_muestras))


        #publisher nodo dar_vueltas
        self.pub_vueltas = self.create_publisher(Int32, '/num_vueltas', 10)


        #Objetos de librerias
        self.bridge = CvBridge()
        self.conversor = Recorte2number()

        # Variables varias
        self.conteo_muestras = 0
        self.estado = 0
        self.enable_muestras = False
        self.numeros = []
        self.colores = []
        self.numero_final = 0
        self.color_final = ""

        
        
        # Inicialización de la cámara y sincornizacion de depth y color
        self.converter = image2recorte()
        self.color_subscription = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_subscription = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.go_button = 0
        self.publish_recorte = self.create_publisher(Image, '/recorte', 10)
        self.publish_recorte_bin = self.create_publisher(Image, '/recorte_bin', 10)


        # Sincronizador de los dos tópicos con una tolerancia en el tiempo
        self.ts = ApproximateTimeSynchronizer(
            [self.color_subscription, self.depth_subscription], 
            queue_size=10, 
            slop=0.1  # Tiempo máximo de diferencia entre los mensajes para ser sincronizados
        )
        self.ts.registerCallback(self.camara_callback)
        self.i = 0

        #Clientes para parar la camara
        self.color_client = self.create_client(SetBool, '/camera/toggle_color')
        self.depth_client = self.create_client(SetBool, '/camera/toggle_depth')

        self.get_logger().info('Esperando servicios...')
        
        # Esperar a que los servicios estén disponibles
        self.color_client.wait_for_service()
        self.depth_client.wait_for_service()



        #Inicio de la FSM
        self.timer = self.create_timer(0.2, self.FSM)
        self.get_logger().info("Brain node Iniciado")
        
    def joy_callback(self, msg: Joy):
        self.go_button = msg.buttons[2]
        # self.get_logger().info(f"Estado del segundo botón: {self.go_button}")
        # self.get_logger().info(f"Estado del segundo botón: {stop_button}")
        # if stop_button == 1: 
            # self.get_logger().info("Botón pulsado: activando kill_nodes")
            # self.kill_nodes()
    
    def camara_callback(self, color_msg, depth_msg):
        try:
            if(self.enable_muestras):
                # Convertir las imágenes de ROS a formato OpenCV
                color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

                # Filtrar los píxeles de color basándonos en el rango de profundidad
                # Definimos un rango de valores de profundidad (en milímetros)

                # Crear una máscara donde la profundidad está dentro del rango especificado
                mask = (depth_image >= self.min_depth) & (depth_image <= self.max_depth)

                # Crear una imagen filtrada de color con los píxeles que están dentro del rango de profundidad
                filtered_color_image = color_image.copy()
                # kernel = np.ones((5, 5), np.uint8)  # El tamaño del kernel controla cuánto se dilata la máscara
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
                dilated_mask = cv2.dilate(mask.astype(np.uint8), kernel, iterations=5)
                filtered_color_image[dilated_mask == 0] = 255  # Ponemos en negro los píxeles fuera del rango
                # Mostrar la imagen filtrada de color
                # cv2.imshow("Filtered Color Image", filtered_color_image)
                cv2.waitKey(0)

                # cv2.waitKey(0)
                # self.get_logger().info("Obteniendo recorte...")
                
                recorte, img_bin = self.converter.obtener_recorte(filtered_color_image, 1)
                if recorte is not None:
                    
                    # cv2.imshow("Recorte", recorte)
                    # imagen_redimensionada = cv2.resize(recorte, (28, 28), interpolation=cv2.INTER_LINEAR)
                    msg = self.bridge.cv2_to_imgmsg(recorte, encoding='bgr8')
                    progreso = len(self.numeros) / self.numero_muestras
                    porcentaje = int(progreso * 100)
                    barra = "#" * (porcentaje // 2)  # Barra de 50 caracteres máx.
                    espacio = " " * (50 - len(barra))  # Relleno para mantener tamaño fijo
                    self.get_logger().info(f"[{barra}{espacio}] {porcentaje}%")
                    # Publica la imagen en el tópico
                    # cv2.imshow("Recorte", recorte)
                    self.publish_recorte.publish(msg)
                    self.publish_recorte_bin.publish(self.bridge.cv2_to_imgmsg(img_bin, encoding='mono8'))
                    # self.get_logger().info("Tratando_imagen...")
                    self.tratar_recorte(recorte)
                    # self.get_logger().info("Imagen tratada con exito!")

        except Exception as e:
            self.get_logger().error('Error al procesar las imágenes: %s' % str(e))
            

    def toggle_color(self, enable: bool):
        """ Llama al servicio para activar/desactivar color """
        request = SetBool.Request()
        request.data = enable
        future = self.color_client.call_async(request)


    def toggle_depth(self, enable: bool):
        """ Llama al servicio para activar/desactivar profundidad """
        request = SetBool.Request()
        request.data = enable
        future = self.depth_client.call_async(request)


    def tratar_recorte(self, image):
        if self.enable_muestras:
            # imagen = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            numero, color = self.conversor.obtener_colorYnum(image)
            if numero is not None:
                self.numeros.append(numero)
            if color is not None:
                self.colores.append(color)
            self.conteo_muestras += 1
            # self.get_logger().info('Color: '+color+'Numero: '+str(numero))
        else:
            pass

    def decision_making(self):
        numeros = self.numeros
        colores = self.colores
        prob_rojo = 0  # Probabilidad de rojo
        prob_azul = 0 # Probabilidad de azul
        for color in colores:
            if color == "Azul":
                prob_azul += 1
            elif color =="Rojo":
                prob_rojo += 1
        prob_rojo /= float(len(colores))
        prob_azul /= float(len(colores))

        frecuencia_por_numero = {i: 0 for i in range(0, 10)}

        for valor in numeros:
            # Filtrar por confianza: solo contar si la confianza supera el umbral
            if valor in frecuencia_por_numero:
                frecuencia_por_numero[valor] += 1  # Ponderar por la confianza

        # Determinar el número con mayor frecuencia ponderada
        numero = max(frecuencia_por_numero, key=frecuencia_por_numero.get)
        # prob_numero = frecuencia_por_numero[numero] / sum(frecuencia_por_numero.values())  # Frecuencia relativa ponderada
        print(self.colores)
        # Determinar el color con mayor probabilidad
        if prob_rojo > prob_azul:
            color = "Rojo"
            prob_color = prob_rojo
        elif prob_azul > prob_rojo:
            color = "Azul"
            prob_color = prob_azul
        else:
            color = "Distractorio"

        return numero, color

    def FSM(self):
        # print("Estado: {}".format(self.estado))
        if self.estado == 0:
            self.enable_muestras = False
            # self.get_logger().info(f"Estado del segundo botón: {self.go_button}")
            if self.go_button == 1:
                self.get_logger().info("Activamos camara color")
                self.toggle_color(True)
                self.get_logger().info("Activamos camara depth")
                self.toggle_depth(True) # Apaga la imagen en color
                self.get_logger().info("Iniciando detección...")
                time.sleep(3)
                self.estado = 1
        elif self.estado == 1:
            self.enable_muestras = True
            if self.conteo_muestras >= self.numero_muestras:
                self.estado = 2
                self.get_logger().info("Desactivamos camara color")
                self.toggle_color(False)
                self.get_logger().info("Desactivamos camara depth")
                self.toggle_depth(False) # Apaga la imagen en color
                self.get_logger().info("Pasamos a calcular estadistica")
            # Cogemos muestras y las tratamos
            pass
        elif self.estado == 2:
            # calculamos estadistica
            self.enable_muestras = False
            self.numero_final, self.color_final = self.decision_making()
            self.get_logger().info("Numeros: {}".format(self.numeros))
            numero_print = str(self.numero_final)
            if self.numero_final == 0:
                numero_print = "No hay numero"
            self.get_logger().info("Numero: "+numero_print+" Color: "+str(self.color_final))
            msg = Int32()
            if self.color_final == "Azul":
                msg.data = -self.numero_final
            elif self.color_final == "Rojo":
                msg.data = self.numero_final
            else:
                msg.data = 0
            self.pub_vueltas.publish(msg)
            self.estado = 3
        elif self.estado == 3:
            # estado de reposo
            time.sleep(10)
            self.numeros = []
            self.colores = []
            self.conteo_muestras = 0
            self.estado = 0
            pass


def main(args=None):
    rclpy.init(args=args)
    brain_percepcion_node = brain_percepcion()
    rclpy.spin(brain_percepcion_node)
    brain_percepcion_node.destroy_node()
    rclpy.shutdown()
        
    
