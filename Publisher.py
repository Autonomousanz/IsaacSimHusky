#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import os

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('husky_velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel', 10)
        self.timer_ = self.create_timer(0.01, self.publish_velocity)
        self.i = 1
        self.msg = Twist() 

    def publish_velocity(self):

        if self.i <= 1000:
            
            self.msg.linear.x = 0.5
            self.msg.angular.z = 0.0  
            

        elif 1000 < self.i <= 4000:
            self.msg.linear.x = 0.2
            self.msg.angular.z = 0.5          
            print(self.i)
        else :
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0

        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing husky velocity: linear=%f, angular=%f' %(self.msg.linear.x, self.msg.angular.z))
        self.i+= 1


def main(args=None):
    rclpy.init(args=args)
    
    husky_velocity_publisher = VelocityPublisher()
    rclpy.spin(husky_velocity_publisher)

    husky_velocity_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
