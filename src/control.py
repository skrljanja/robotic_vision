#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

class control:
    
    def __init__(self): 
        self.detect_joint_image = rospy.Subscriber('/robot/joint_states', Float64MultiArray, self.callback)
    
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


