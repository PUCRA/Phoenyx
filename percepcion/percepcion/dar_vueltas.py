import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu  # 📌 Cambiado de Twist a Imu
from geometry_msgs.msg import Twist
from percepcion.pid import pid
import time
import math
import tf_transformations as tf

class DarVueltas(Node):
    def __init__(self):
        super().__init__('dar_vueltas')
        self.subscriber_ = self.create_subscription(Int32, '/num_vueltas', self.callback, 10)
        self.subs_imu = self.create_subscription(Imu, '/imu/data', self.imu_update, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controlador = pid(0.06, 0.03, 0.0, 0)
        self.controlador.set_max_val(3)
        self.first_iteration = True #esta variable determina si es la primera iteración del codigo (en func. imu_update)
        self.rotation = 0 
        self.sentido_rotacion = -1 #1 sentido horario, -1 sentido antihorario
        self.roll = 0
        self.num_vueltas = 0
        self.prev_angle = 0
        self.ini_rotation = 0
        self.setpoint = 0
        self.get_logger().info('Dar vueltas node started')

    def callback(self, msg):
        if self.num_vueltas == 0 and msg.data != 0:
            self.get_logger().info('Dando {} vueltas'.format(msg.data))
            self.num_vueltas = msg.data
            self.timer = self.create_timer(0.02, self.timer_callback)
            self.setpoint = self.rotation+self.num_vueltas*360.0
            self.controlador.set_setpoint(self.setpoint)#en grados
            self.ini_rotation = math.degrees(self.roll)
            self.get_logger().info("Setpoint: {}".format(self.rotation+self.num_vueltas*360.0))


    def timer_callback(self):
        #self.get_logger().info('Ángulo actual: {}'.format(self.rotation))
        value = self.controlador.update(self.rotation*self.sentido_rotacion, 0.02)
        if(abs(self.rotation-self.setpoint) < 300):
            self.controlador.set_setpoint(self.ini_rotation)
            value = self.controlador.update(math.degrees(self.roll), 0.02)
        # else:
            
        

        Twist_msg = Twist()
        Twist_msg.angular.y = float(value)*self.sentido_rotacion
        Twist_msg.linear.x = 0.0

        if abs(self.controlador.get_error()) < 0.5:
            self.get_logger().info('Deteniendo robot')
            self.rotation = 0 # resetea la rotación para poder hacer varias veces girar n vueltas
            self.num_vueltas = 0
            Twist_msg.angular.y = 0.0
            self.timer.destroy()

        
        # self.get_logger().info('Publicando mensaje')
        self.get_logger().info('Rotacion: {}'.format(self.rotation))
        self.get_logger().info('Roll: {}'.format(math.degrees(self.roll)))
        self.get_logger().info('Error: {}'.format(self.controlador.get_error()))
        self.pub.publish(Twist_msg)

    def imu_update(self, msg):       
        
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, pitch, yaw) = tf.euler_from_quaternion(orientation_list)

        if(self.first_iteration): #en la primera iteración 
            self.prev_angle = self.roll
            self.first_iteration = False
         
        
        delta_roll = (self.roll - self.prev_angle)*(363.75/360)
        if delta_roll > math.pi: # para cuando pasa ej. de 0 a 360 (delta_roll = 360 > 180)
            delta_roll -= 2 * math.pi
        elif delta_roll < -math.pi: # para cuando pasa ej. de 360 a 0 (delta_roll = -360 < -180)
            delta_roll += 2 * math.pi
        

        
        self.rotation += math.degrees(delta_roll)
        
       
        self.prev_angle = self.roll

def main(args=None):
    rclpy.init(args=args)
    dar_vueltas = DarVueltas()
    rclpy.spin(dar_vueltas)
    dar_vueltas.destroy_node()
    rclpy.shutdown()