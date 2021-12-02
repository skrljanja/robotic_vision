#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys

class control:
    
    def __init__(self): 
        
        rospy.init_node('image_processing', anonymous=True)
        
        self.bridge = CvBridge()
        self.cv_image1 = Image()
        self.cv_image2 = Image()
        self.previous_time= np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        self.joint1 = rospy.Subscriber('/joint_angle_1', Float64, self.callback1)
        self.joint3 = rospy.Subscriber('/joint_angle_3', Float64, self.callback3)
        self.joint4 = rospy.Subscriber('/joint_angle_4', Float64, self.callback4)
    
    
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback5)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback6)
        self.cv_image1 = Image()
        self.cv_image2 = Image()
    
    
        self.joint1_control_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size = 10)
        self.joint3_control_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size = 10)
        self.joint4_control_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size = 10)
                
        self.target_pos_sub = rospy.Subscriber('/target_pos', Float64MultiArray, self.callback_traj)
        #self.previous_time = rospy.get_time()
        self.error = np.array([0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0,0.0], dtype='float64')
        
        self.joint_angle1 = Float64()
        self.joint_angle3 = Float64()
        self.joint_angle4 = Float64()

        self.joints_test = Float64MultiArray()
        
        
       

    def forward_kinematics(self):
        """j1 = self.joints_test.data[0]
        j3 = self.joints_test.data[2]
        j4 = self.joints_test.data[3]
        """
        
        j1 = (self.joint_angle1.data)
        j3 = (self.joint_angle3.data)
        j4 = (self.joint_angle4.data)
        s1 = np.sin(j1)
        s3 = np.sin(j3)
        s4 = np.sin(j4)
        c1 = np.cos(j1)
        c3 = np.cos(j3)
        c4 = np.cos(j4)

        end_effector_x = 2.8 * ( c1*s4  + s1*s3*c4) + 3.2*s1*s3
        end_effector_y = 2.8 * ( s1*s4 - c1*s3*c4) - 3.2*c1*s3
        end_effector_z = 2.8 * ( c3*c4) + 3.2*c3 + 4
        
        end_effector = np.array([end_effector_x, end_effector_y, end_effector_z])
        return end_effector
    
    
    def detect_red(self,image):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_green(self,image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])


    def detect_blue(self,image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])



    def detect_yellow(self,image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])



    def pixel2meter(self,image):
        circle2Pos = self.detect_yellow(image)
        circle1Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos)**2)
        return 4 / np.sqrt(dist)
  
    def calculate_jacobian(self, j1, j3, j4):
        s1 = np.sin(j1)
        s3 = np.sin(j3)
        s4 = np.sin(j4)
        c1 = np.cos(j1)
        c3 = np.cos(j3)
        c4 = np.cos(j4)
        jacobian = np.array([ [(2.8*(-s1*s4 + c1*s3*c4) + 3.2*s1*s3), (2.8*(s1*c3*c4) + 3.2*s1*s3), (2.8*(c1*c4 - s1*s3*s4))],
                               [(2.8*(c1*s4 + s1*s3*c4) + 3.2*s1*s3), (2.8*(-c1*c3*c4) - 3.2*c1*c3), (2.8*(s1*c4 + c1*s3*s4))], 
                               [0, (2.8*(-s3*c4) - 3.2*s3), (2.8*(c3*s4))]])
        return jacobian
    

    def control_open(self):
        current_time = rospy.get_time()
        dt = current_time - self.previous_time
        self.previous_time = current_time
        
        q = np.array([self.joint_angle1.data, self.joint_angle3.data, self.joint_angle4.data]) # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(q[0], q[1], q[2]))  # calculating the psudeo inverse of Jacobian
        position = self.end_effector.data

        target = self.target_pos.data
        # estimate derivative of desired trajectory
        self.error = (target - position)/dt
        q_d = q + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
        return q_d
    
    
    def callback1(self, data):
        self.joint_angle1 = Float64()
        self.joint_angle1= data
        
    def callback3(self, data):
        self.joint_angle3 = Float64()
        self.joint_angle3 = data
        
    def callback4(self, data):
        self.joint_angle4 = Float64()
        self.joint_angle4 = data
        
        
    def callback5(self, image):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            

        self.end_effector_y = Float64()
        self.end_effector_z = Float64()
      
        a = self.pixel2meter(self.cv_image1)
        pos = self.detect_red(self.cv_image1)
        pos_center = self.detect_green(self.cv_image2)  

        self.end_effector_y.data = a*(pos[0] - pos_center[0])
        self.end_effector_z.data = -a*(pos[1] - pos_center[1])

        
    def callback6(self, image):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        self.end_effector_x = Float64()
        a = self.pixel2meter(self.cv_image2)
        pos_end = self.detect_red(self.cv_image2)  
        pos_center = self.detect_green(self.cv_image2)  
        self.end_effector_x.data = a * (pos_end[0] - pos_center[0]) 
        
      
        #self.callback()
        
    def callback_traj(self, data):
        self.target_pos = Float64MultiArray()
        self.target_pos = data
        self.callback()

    def callback(self):
        print("here")
        end_effector_pos_FK = self.forward_kinematics()
        end_effector_pos_vision = np.array([self.end_effector_x.data, self.end_effector_y.data, self.end_effector_z.data])
        
        # compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
        x_e = self.forward_kinematics()

        self.end_effector=Float64MultiArray()
        self.end_effector.data=  end_effector_pos_vision

        # send control commands to joints (lab 3)
        q_d = self.control_open()
        self.joint1_angle=Float64()
        self.joint1_angle.data= q_d[0]
        self.joint3_angle=Float64()
        self.joint3_angle.data= q_d[1]
        self.joint4_angle=Float64()
        self.joint4_angle.data= q_d[2]
        


        
        try:
            self.joint1_control_pub.publish(self.joint1_angle)
            self.joint3_control_pub.publish(self.joint3_angle)
            self.joint4_control_pub.publish(self.joint4_angle)
        except CvBridgeError as e:
            print(e)

def main(args):
  ctrl = control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
