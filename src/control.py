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
        
        self.target_pos = rospy.Subscriber('/target_control/target_pos', Float64, queue_size = 10)
        self.previous_time = rospy.get_time()

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
    
    def calculate_jacobian(self, j1, j3, j4):
        s1 = np.sin(j1)
        s3 = np.sin(j3)
        s4 = np.sin(j4)
        c1 = np.cos(j1)
        c3 = np.cos(j3)
        c4 = np.cos(j4)
        jacobian = np.array([ [(2.8*(-c1*c3*s4 - c1*c3*c4) + 3.2*c1*s3), (2.8*(s1*s3*s4 + s1*s3*c4) - 3.2*s1*c3), (2.8*(-s1*c3*c4 + s1*c3*s4))],
                               [(2.8*(-s1*c3*s4 + s1*c3*c4) - 3.2*s1*s3), (2.8*(-c1*s3*s4 + c1*s3*c4) + 3.2*c1*c3), (2.8*(c1*c3*c4 + c1*c3*s4))], 
                               [0, (2.8*(c3*s4 - s3*c4) + 3.2*c3), (2.8*(s3*c4 - c3*s4))]])
        return jacobian
    
    def control_open(self,image):
        current_time = rospy.get_time()
        dt = current_time - self.previous_time
        self.previous_time = current_time
        
        q = self.detect_joint_angles(image) # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
        position = self.detect_end_effector(image)

        target = self.target_pos()
        # estimate derivative of desired trajectory
        self.error = (target - position)/dt
        q_d = q + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
        return q_d


