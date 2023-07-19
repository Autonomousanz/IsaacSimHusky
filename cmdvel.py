#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np



def publisher():
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    rospy.init_node('cmd_vel_array_publisher')
    rate = rospy.Rate(100)
  
    vel = Twist()
    time_0 = rospy.Time.now().to_sec()
    time_1 = rospy.Time.now().to_sec()
    time_diff = time_1-time_0
    i=1
    while not rospy.is_shutdown():
        if i < 6000:
            vel.linear.x = 0.2
            vel.angular.z = -0.00002*i
       

        else:
            vel.linear.x = 0
            vel.angular.z = 0
        pub.publish(vel)
        i = i+1
        print(i)
        rate.sleep()
          


if __name__ == '__main__':
    publisher()