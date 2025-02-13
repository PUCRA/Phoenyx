#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from  geometry_msgs.msg import Twist
import pygame

class XboxController(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        pygame.init()
        pygame.joystick.init()
        num_joysticks = pygame.joystick.get_count()
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.02, self.controller_callback)


        if num_joysticks > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            print("Controller connected:", self.controller.get_name())
        else:
            print("No controller detected.")
            exit()
    
    def controller_callback(self):
        pygame.event.get()
        vx = -self.controller.get_axis(1)
        vy = -self.controller.get_axis(0)
        vangular = -self.controller.get_axis(3)
        if abs(vx) < 0.1:
            vx = 0
        if abs(vy) < 0.1:
            vy = 0
        if abs(vangular) < 0.1:
            vangular = 0
        msg = Twist()
        msg.linear.x = vx*5.0
        msg.linear.y = vy*5.0
        msg.angular.z = vangular*5.0
        print(msg.linear.x, msg.linear.y, msg.angular.z)
        self.pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    controller = XboxController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__=="__main__":
    main()
