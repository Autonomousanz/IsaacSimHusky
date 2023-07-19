#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np



def publisher():
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    rospy.init_node('cmd_vel_array_publisher')
    rate = rospy.Rate(100)
  
    vel = Twist()
    i=1
    while not rospy.is_shutdown():
        if i <= 1000:
            
            vel.linear.x = 0.5
            vel.angular.z = 0.0  

        elif 1000 < i <= 4000:
            vel.linear.x = 0.2
            vel.angular.z = 0.5   

        else:
            vel.linear.x = 0
            vel.angular.z = 0
            
        pub.publish(vel)
        i = i+1
        print(i)
        rate.sleep()
          


if __name__ == '__main__':
    publisher()