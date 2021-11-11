#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64

class control:
    
    def __init__(self): 
        self.joint1 = rospy.Subscriber('/joint_angle_1', Float64, self.callback)
        self.joint3 = rospy.Subscriber('/joint_angle_3', Float64, self.callback)
        self.joint4 = rospy.Subscriber('/joint_angle_4', Float64, self.callback)

        self.joint1_control = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size = 10)
        self.joint1_control = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size = 10)
        self.joint1_control = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size = 10)

    def forward_kinematics(self,image):
        joints = self.detect_joint_angles(image)
        end_effector_x = (2.8 * ( - np.sin(joints[0])*np.cos(joints[2])*np.sin(joints[3]) 
                                    - np.sin(joints[0]) * np.sin(joints[2]) * np.cos(joints[3]))
                                    - 3.2 * np.sin(joints[0]) * np.sin(joints[2]))
        end_effector_y = (2.8 * (np.cos(joints[0])*np.cos(joints[2])*np.sin(joints[3]) 
                                    - np.cos(joints[0]) * np.cos(joints[2]) * np.cos(joints[3]))
                                    + 3.2 * np.cos(joints[0]) * np.sin(joints[2]))
        end_effector_z = (2.8 * (np.sin(joints[2])*np.sin(joints[3]) 
                                    + np.cos(joints[2]) * np.cos(joints[3]))
                                    + 3.2 * np.sin(joints[2])
                                    + 4)
        
        end_effector = np.array([end_effector_x, end_effector_y, end_effector_z])
        return end_effector


