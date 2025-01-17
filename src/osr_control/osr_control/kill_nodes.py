#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from  geometry_msgs.msg import Twist


class node_killer(Node):
    def __init__(self):
        super().__init__("node_killer")
        
        self.battery_subscription = self.create_subscription(
            BatteryState,                # Tipo de mensaje
            'battery_state',             # Nombre del tópico al que nos suscribimos
            self.battery_callback,       # Callback que se ejecutará cuando se reciba un mensaje
            10                            # QoS (Quality of Service). El valor de 10 es común.
          
        ) 
        self.voltage_threshold = 12.0  # Define el umbral de voltaje
    def battery_callback(self, msg):
        # Este es el callback que se ejecutará cuando se reciba un mensaje en el tópico 'battery_state'
        voltage = msg.voltage
        current = msg.current
        self.get_logger().info(f"Voltaje: {voltage}, Corriente: {current}")

       
    if voltage < self.voltage_threshold:
            self.get_logger().warn(f"Voltaje bajo detectado: {voltage}. Apagando todos los nodos.")
            self.shutdown_all_nodes()
  
    def shutdown_all_nodes(self):
        # Lógica para apagar todos los nodos
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__=="__main__":
    main()
