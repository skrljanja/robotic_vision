#!/usr/bin/env python3

import rospy
import roslib
import sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


class vision2:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.cv_image1 = Image()
    self.cv_image2 = Image()
    
    # image1
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    
    # image2
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    
    # joints
    # (btw are you sure we define each joint separately and not like in lab 1? "self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)")
    # self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    self.joint1_pub = rospy.Publisher('joint_angle_1', Float64, queue_size = 10)
    self.joint3_pub = rospy.Publisher('joint_angle_3', Float64, queue_size = 10)
    self.joint4_pub = rospy.Publisher('joint_angle_4', Float64, queue_size = 10)
    
    
    self.joint1 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()
    

    
  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # this if statement handles the case where the joint is obscured. returns junk value coordinates
      height, width = image.shape[:2]
      if height == 0:
          return np.array([-1, -1])
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob#if M['m00'] == 0:
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      height, width = image.shape[:2]

      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      height, width = image.shape[:2]
      if height == 0:
          return np.array([-1, -1])
      
      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])
  

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      
      height, width = image.shape[:2]
      if height == 0:
          return np.array([-1, -1])
      
      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])
  
  def unit_vector(self, vector):
    return (vector / np.linalg.norm(vector))
  
  def angle_between(self, v1, v2):

    v1_u = self.unit_vector(v1)
    v2_u = self.unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

  def project_vector(self, vector):        
      normal = np.array([0,1,0])       
   
      n_norm = np.sqrt(sum(normal**2))    
      projection = (np.dot(vector, normal)/n_norm**2)*normal
  
      print (projection)
      return projection

      
  def get_link1_vector(self, image1, image2):
        bottom_im1 = self.detect_green(image1)
        bottom_im2 = self.detect_green(image2)
        top_im1 = self.detect_yellow(image1)
        top_im2 = self.detect_yellow(image2)
        
        x = top_im2[0] - bottom_im2[0]
        y = top_im1[0] - bottom_im1[0]
        z = top_im1[1] - bottom_im1[1]
        
        return np.array([x,y,z])
        
  def get_link2_vector(self, image1, image2):
        top_im1 = self.detect_blue(image1)
        top_im2 = self.detect_blue(image2)
        bottom_im1 = self.detect_yellow(image1)
        bottom_im2 = self.detect_yellow(image2)
        if (top_im1[0] == -1):
            top_im1 = bottom_im1
        if (top_im2[0] == -1):
            top_im2 = bottom_im2
    
        
        x = top_im2[0] - bottom_im2[0]
        y = top_im1[0] - bottom_im1[0]
        z = top_im1[1] - bottom_im1[1]
        
        return np.array([x,y,z])
      

  def get_link3_vector(self, image1, image2):
        bottom_im1 = self.detect_blue(image1)
        bottom_im2 = self.detect_blue(image2)
        top_im1 = self.detect_red(image1)
        top_im2 = self.detect_red(image2)
        
        if (bottom_im1[0] == -1):
            bottom_im1 = self.detect_yellow(image1)
        if (bottom_im2[0] == -1):
            bottom_im2 = self.detect_yellow(image2)
        if (top_im1[0] == -1):
            top_im1 = bottom_im1
        if (top_im2[0] == -1):
            top_im2 = bottom_im2
        # add dealing with -1
        
        x = top_im2[0] - bottom_im2[0]
        y = top_im1[0] - bottom_im1[0]
        z = top_im1[1] - bottom_im1[1]
        
        return np.array([x,y,z])
    
  
    
  def detect_joint_angles(self, image1, image2):
      v1 = self.get_link1_vector(image1, image2)
      v2 = self.get_link2_vector(image1, image2)
      v3 = self.get_link3_vector(image1, image2)
      

      ja4 = self.angle_between(v2, v3)
      ja3 = self.angle_between(v1, v2) 
      if (self.angle_between(v2, np.array([0,-1,0])) > np.pi/2):
          ja3 = - ja3
    

      v3_dummy = np.array([np.sin(ja4), 0, np.cos(ja4)])

      
      ja1 = self.angle_between(np.array([0, -1, 0]), np.array([v2[0], v2[1], 0]))
      if ((ja3 < 0.2) & (ja3 > 0.2)):
          ja1 = self.angle_between(v3, v3_dummy)
          
      return np.array([ja1, ja3, ja4])
      
      
  
    
  def callback1(self, data): 
      try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
            print(e)


  def callback2(self, data): 
      try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
            print(e)

      self.callback()
      

  def callback(self):
        
    
        angles = self.detect_joint_angles(self.cv_image1, self.cv_image2)

        self.joint1 = Float64()
        self.joint1.data = angles[0]

        self.joint3 = Float64()
        self.joint3.data = angles[1]
    
        self.joint4 = Float64()
        self.joint4.data = angles[2]
    
        try:
            self.joint1_pub.publish(self.joint1)
            self.joint4_pub.publish(self.joint4)
            self.joint3_pub.publish(self.joint3)
        except CvBridgeError as e:
            print(e)
            
def main(args):
  vis2 = vision2()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
    

