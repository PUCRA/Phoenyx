import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import numpy as np
import math

class Nav2WaypointExplorer(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_explorer')
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Suscripciones
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.mapa_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publicador para /goal_pose
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Variables de estado
        self.robot_pose = None  
        self.mapa_actual = None  
        self.map_counter = 0  # Contador de mapas recibidos
        self.initial_pose = None  # Guardar la posición inicial

        # Temporizador para procesar el mapa cada 1 segundo
        self.timer = self.create_timer(3.0, self.procesar_mapa)
        
        self.get_logger().info("Escuchando mensajes en /map y /odom...")

    def odom_callback(self, msg):
        """Guarda la posición y orientación del robot desde /odom"""
        self.robot_pose = msg.pose.pose  # Guarda la pose completa
        if self.initial_pose is None:
            self.initial_pose = self.robot_pose  # Guardar la posición inicial
        # self.get_logger().info("Pose del robot actualizada.")

    def mapa_callback(self, msg):
        """Guarda el último mapa recibido y cuenta los mapas recibidos"""
        self.mapa_actual = msg
        self.map_counter += 1  # Incrementar contador
        # self.get_logger().info(f"Mapa {self.map_counter} recibido con timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def procesar_mapa(self):
        """Procesa el último mapa recibido si existe"""
        if self.mapa_actual is None or self.robot_pose is None:
            self.get_logger().info("Esperando datos de mapa y pose.")
            return

        self.get_logger().info("Procesando mapa en bucle...")
        # Extraer datos del mapa
        width = self.mapa_actual.info.width
        height = self.mapa_actual.info.height
        resolution = self.mapa_actual.info.resolution
        origin = self.mapa_actual.info.origin.position

        # Convertir el mapa en un array de numpy
        mapa_array = np.array(self.mapa_actual.data).reshape((height, width))

        # Detectar fronteras basadas en la pose del robot
        waypoints = self.detectar_fronteras(mapa_array, resolution, origin)

        if waypoints:
            self.get_logger().info(f"Se encontraron {len(waypoints)} fronteras.")
            self.publicar_waypoint(waypoints[0])  # Enviar solo el primer waypoint
        else:
            self.get_logger().info("No se encontraron fronteras adecuadas.")
            self.get_logger().info(f"Mapa completo: \n{mapa_array}")
            if self.initial_pose:
                self.get_logger().info("Mapa completo, regresando a la posición inicial.")
                # Se asegura de enviar el waypoint (0, 0) o la posición inicial del robot
                self.publicar_waypoint((0.0, 0.0))  # Puedes reemplazar (0, 0) por self.initial_pose si prefieres.
            else:
                self.get_logger().info("Posición inicial no disponible, enviando a (0, 0).")
                self.publicar_waypoint((0,0, 0.0))  # Enviar a (0, 0) si no se encuentra la posición inicial.
            self.destroy_node()  # Cerrar el nodo si no se encuentran fronteras

    def detectar_fronteras(self, mapa, resolution, origin):
        """Detecta fronteras teniendo en cuenta la pose del robot"""
        if self.robot_pose is None:
            return []

        # Posición y orientación del robot
        rx, ry = self.robot_pose.position.x, self.robot_pose.position.y
        _, _, yaw = self.quaternion_to_euler(self.robot_pose.orientation)

        # Listas de puntos de interés
        puntos_100 = []
        fronteras_candidatas = []

        # Recorrer el mapa para obtener los puntos de interés
        for y in range(mapa.shape[0]):
            for x in range(mapa.shape[1]):
                wx = origin.x + x * resolution
                wy = origin.y + y * resolution

                if mapa[y, x] == 100:
                    puntos_100.append((wx, wy))

        # Buscar puntos frontera
        for y in range(1, mapa.shape[0] - 1):
            for x in range(1, mapa.shape[1] - 1):
                if mapa[y, x] == -1:  # Celda desconocida
                    vecinos = [mapa[y-1, x], mapa[y+1, x], mapa[y, x-1], mapa[y, x+1]]
                    
                    if any(v == 0 for v in vecinos):  # Tiene un vecino libre (0)
                        wx = origin.x + x * resolution
                        wy = origin.y + y * resolution

                        # Filtrar puntos que estén a máximo 5 metros del robot
                        distancia = np.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
                        if distancia > 7.0:
                            continue

                        # Filtrar solo puntos delante del robot
                        if not self.esta_detras(rx, ry, yaw, wx, wy):
                            
                            # Contar cuántos puntos 100 están a menos de 0.4m
                            puntos_cercanos_04m = sum(
                                np.sqrt((wx - px) ** 2 + (wy - py) ** 2) < 0.2
                                for px, py in puntos_100
                            )

                            # Si hay más de 2 puntos 100 a menos de 0.4m, descartar
                            if puntos_cercanos_04m > 2:
                                continue  

                            # Contar cuántos puntos 100 están en el rango 0.7 - 0.8 m
                            puntos_cercanos_07_08m = sum(
                                0.7 <= np.sqrt((wx - px) ** 2 + (wy - py) ** 2) <= 0.8
                                for px, py in puntos_100
                            )

                            if puntos_cercanos_07_08m > 0:  # Solo guardar si tiene puntos 100 en ese rango
                                fronteras_candidatas.append((wx, wy, puntos_cercanos_07_08m))

        # Seleccionar la mejor frontera (máximo de puntos 100 en 0.7 - 0.8 m)
        if fronteras_candidatas:
            mejor_punto = max(fronteras_candidatas, key=lambda p: p[2])
            self.get_logger().info(f"Mejor frontera seleccionada: {mejor_punto[:2]}, con {mejor_punto[2]} puntos 100 en 0.7 - 0.8 m.")
            return [mejor_punto[:2]]  # Retornar solo coordenadas (x, y)

        self.get_logger().info("No se encontraron fronteras que cumplan los requisitos.")
        return []


    def esta_detras(self, rx, ry, yaw, px, py):
        """Determina si el punto (px, py) está detrás del robot"""
        dx = px - rx
        dy = py - ry
        angulo_punto = math.atan2(dy, dx)  # Ángulo hacia el punto
        diferencia = abs(math.atan2(math.sin(angulo_punto - yaw), math.cos(angulo_punto - yaw)))
        return diferencia > math.pi / 2  # Si la diferencia angular es mayor a 90° está detrás

    def quaternion_to_euler(self, q):
        """Convierte un cuaternión en ángulos de Euler (roll, pitch, yaw)"""
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def publicar_waypoint(self, waypoint):
        self.get_logger().info(f"Publicando waypoint: {waypoint}")
        pose_msg = self.create_pose(*waypoint)
        self.goal_pose_pub.publish(pose_msg)
        self.enviar_waypoint(pose_msg)

    def enviar_waypoint(self, pose):
        self.get_logger().info(f"Enviando waypoint a la acción de navegación: {pose.pose.position.x}, {pose.pose.position.y}")
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses.append(pose)
        #self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

def main():
    rclpy.init()
    nodo = Nav2WaypointExplorer()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
