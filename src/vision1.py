#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class vision1: 
    
    def __init__(self):
        self.image1_sub = rospy.Subscriber('camera1/robot/image_raw', Image, self.callback)
        self.image2_sub = rospy.Subscriber('camera2/robot/image_raw', Image, self.callback)
        
        self.joint2 = rospy.Publisher('/joint_angle_2', Float64, queue_size = 10)
        self.joint3 = rospy.Publisher('/joint_angle_3', Float64, queue_size = 10)
        self.joint4 = rospy.Publisher('/joint_angle_4', Float64, queue_size = 10)
