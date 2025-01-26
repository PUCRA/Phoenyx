import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')
        self.get_logger().info('Nodo KillerNode inicializado.')
        
        # Timer para verificar nodos en ejecución cada 5 segundos
        self.timer = self.create_timer(5.0, self.kill_nodes)

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
        nodes = self.get_active_nodes()
        if not nodes:
            self.get_logger().info('No se encontraron nodos activos.')
            return

        self.get_logger().info(f'Nodos activos encontrados: {nodes}')

        for node in nodes:
            if node == '/killer_node':
                continue

            self.get_logger().info(f"Intentando detener el nodo: {node}")

            # Encuentra el proceso asociado al nodo y lo termina
            try:
                result = subprocess.run(
                    ['pgrep', '-f', node],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                if result.returncode == 0:
                    # Obtén los IDs de los procesos
                    pids = result.stdout.strip().split('\n')
                    for pid in pids:
                        self.get_logger().info(f"Matando proceso con PID: {pid}")
                        os.kill(int(pid), signal.SIGTERM)  # Envía la señal de terminación
                    self.get_logger().info(f"Nodo {node} detenido exitosamente.")
                else:
                    self.get_logger().warning(f"No se encontró el proceso para el nodo {node}.")
            except Exception as e:
                self.get_logger().error(f"Error al intentar detener el nodo {node}: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()