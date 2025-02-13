import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import BatteryState, Joy 

class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')
        self.get_logger().info('Nodo KillerNode inicializado.')
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        
        
        self.subscription  # Evita que el objeto sea eliminado por el recolector de basura
        self.joy_subscription = self.create_subscription(
            Joy,                   # Tipo de mensaje para el joystick
            '/joy',                # Tópico de los mensajes del joystick
            self.joy_callback,     # Callback para procesar el mensaje de Joy
            10                     # Tamaño de la cola de mensajes
        )
    
        # Timer para verificar nodos en ejecución cada 5 segundos
        self.timer = self.create_timer(2.0, self.kill_nodes)
        self.iter = 0
        self.nodes_whitelist = ["/killer_node"]
        self.whitelist_pid = [os.getpid()]
        result3 = subprocess.run(
                ['pgrep', '-f', "ros2 run phoenyx_nodes killer_node"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
        if result3.returncode == 0:
            self.whitelist_pid.append(int(result3.stdout.strip().split('\n')[0]))
        self.stop_Xbox = 0

    def joy_callback(self, msg: Joy):
        self.stop_Xbox = msg.buttons[1]
        self.get_logger().info(f"Estado del segundo botón: {self.stop_Xbox}")
        self.get_logger().info("Botón pulsado: activando kill_nodes")
        
    

    def battery_callback(self, msg):
        self.voltage = msg.voltage
        self.current = msg.current
        self.get_logger().info(f'Voltaje: {msg.voltage} V, Corriente: {msg.current} A')
        if self.voltage <= 14.8 or self.voltage >= 18.0:
            self.timer = self.create_timer(2.0, self.kill_nodes)
        else:
            self.destroy_timer(self.timer)
            


    def get_active_nodes(self):
        """
        Devuelve una lista de los nodos activos en el sistema usando `ros2 node list`.
        """
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                return nodes
            else:
                self.get_logger().error(f"Error al obtener nodos: {result.stderr}")
                return []
        except Exception as e:
            self.get_logger().error(f"Error ejecutando ros2 node list: {str(e)}")
            return []

    def kill_nodes(self):
        """
        Encuentra nodos activos y los detiene.
        """
        nodes =  []
        nodes = self.get_active_nodes()
        # if nodes in self.nodes_whitelist:
        #     return

        for node in nodes:
            if node in self.nodes_whitelist:
                continue
            self.get_logger().info(f"Intentando detener el nodo: {node}")
            busqueda = subprocess.run(
                ['pgrep', '-f', node[1:]],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            if busqueda.returncode == 0:
                pids = busqueda.stdout.strip().split('\n')
                for pid in pids:
                    if pid != '' and pid not in self.whitelist_pid:
                        self.get_logger().info(f"Matando {node} proceso con PID: {pid}")
                        os.kill(int(pid), signal.SIGTERM)
            else:
                self.get_logger().warning(f"Nodo {node} es remoto, añadiendo a la whitelist")
                self.nodes_whitelist.append(node)
        result2 = subprocess.run(
            ['pgrep', '-f', "ros2 launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
            )
        pids = []
        if result2.returncode == 0:
            print(result2)
            pids = result2.stdout.strip().split('\n')
            self.get_logger().info("Launch files encontrados, matando...")
            for pid in pids:
                print(pid)
                if pid != '' and pid not in self.whitelist_pid:
                    self.get_logger().info(f"Matando proceso con PID: {pid}")
                    os.kill(int(pid), signal.SIGTERM)

        print(f"whitelist: {self.whitelist_pid}")



def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    # Manejo adecuado de señales para permitir interrupción de teclado
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
#    
#import os
#import signal
#import subprocess
#import rclpy
#from rclpy.node import Node
#from rclpy.executors import ExternalShutdownException
#from sensor_msgs.msg import BatteryState
#
#class KillerNode(Node):
#    def __init__(self):
#        super().__init__('killer_node')
#        self.get_logger().info('Nodo KillerNode inicializado.')
#        self.subscription = self.create_subscription(
#            BatteryState,
#            '/battery_state',
#            self.battery_callback,
#            10)
#        # self.subscription  # Evita que el objeto sea eliminado por el recolector de basura
#
#        
#        # Timer para verificar nodos en ejecución cada 5 segundos
#        # self.timer = self.create_timer(2.0, self.kill_nodes)
#        self.iter = 0
#        self.nodes_whitelist = ["/killer_node"]
#        self.whitelist_pid = [os.getpid()]
#        result3 = subprocess.run(
#                ['pgrep', '-f', "ros2 run phoenyx_nodes killer_node"],
#                stdout=subprocess.PIPE,
#                stderr=subprocess.PIPE,
#                text=True
#            )
#        if result3.returncode == 0:
#            self.whitelist_pid.append(int(result3.stdout.strip().split('\n')[0]))
#
#    def battery_callback(self, msg):
#        self.voltage = msg.voltage
#        self.current = msg.current
#        self.get_logger().info(f'Voltaje: {msg.voltage} V, Corriente: {msg.current} A')
#        if self.voltage <= 14.8 or self.voltage >= 18.0:
#            self.timer = self.create_timer(2.0, self.kill_nodes)
#        else:
#            self.destroy_timer(self.timer)
#            