#!/usr/bin/env python3

import roslib
import rospy
from std_msgs.msg import Float64
import numpy as np 

class sin_trajectory1:
    
    def __init__(self):
        rospy.init_node('sin_trajectory1', anonymous = True)
        self.joint2_publisher = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size = 10)
        self.joint3_publisher = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size = 10)
        self.joint4_publisher = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size = 10)
        self.start_time = rospy.get_time()
        self.rate = rospy.Rate(10)
        
    def trajectory(self):
        
        t = rospy.get_time() - self.start_time
        J2 = float((np.pi/2) * np.sin((np.pi/15) * t))
        J3 = float((np.pi/2) * np.sin((np.pi/20) * t))
        J4 = float((np.pi/2) * np.sin((np.pi/18) * t))

        return np.array([J2, J3, J4])
        
    def callback(self):
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        
        new_position = self.trajectory()
        
        self.joint2.data = new_position[0]
        self.joint3.data = new_position[1]
        self.joint4.data = new_position[2]

        self.joint2_publisher.publish(self.joint2)
        self.joint3_publisher.publish(self.joint3)
        self.joint4_publisher.publish(self.joint4)
        self.rate.sleep()
        
            
if __name__ == '__main__':
    p = sin_trajectory1()
    while not rospy.is_shutdown():
        p.callback()

            
