#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

# imports from image files
import roslib
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class vision1:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    self.cv_image1 = Image()
    self.cv_image2 = Image()
    
    # image1
    #self.cv_image1 = Image()
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    
    # image2
    #self.cv_image2 = Image()
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    
    # joints
    # (btw are you sure we define each joint separately and not like in lab 1? "self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)")
    # self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    self.joint2_pub = rospy.Publisher('joint_angle_2', Float64, queue_size = 10)
    self.joint3_pub = rospy.Publisher('joint_angle_3', Float64, queue_size = 10)
    self.joint4_pub = rospy.Publisher('joint_angle_4', Float64, queue_size = 10)
    
    	
  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      height, width = image.shape[:2]
      if (height == 0) or (M['m00'] == 0):
      	return np.array([-1,-1])
      else:
	# Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])


  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      height, width = image.shape[:2]
      if (height == 0) or (M['m00'] == 0):
      	return np.array([-1,-1])
      else:
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      height, width = image.shape[:2]
      if (height == 0) or (M['m00'] == 0):
      	return np.array([-1,-1])
      else:
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      height, width = image.shape[:2]
      if (height == 0) or (M['m00'] == 0):
      	return np.array([-1,-1])
      else:
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])
        
        
  def unit_vector(self, vector):
    return (vector / np.linalg.norm(vector))
  
  def angle_between(self, v1, v2, in_3d, ja2):
    v1_u = self.unit_vector(v1)
    v2_u = self.unit_vector(v2)
    
    # handle cases with joints 2 and 3
    if in_3d == False:
      if ja2:
        v1_u = [v1_u[0], v1_u[2]]
        v2_u = [v2_u[0], v2_u[2]]
      else:
        v1_u = [v1_u[1], v1_u[2]]
        v2_u = [v2_u[1], v2_u[2]]
        
    # handling cases when angles for joints 2 and 3 should be negative
    if ((v2[1] > 0) and (in_3d == False) and (ja2 == False)) or ((v2[0] < 0) and (in_3d == False) and (ja2 == True)):
      return -np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    else:
      return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

      
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
        
        # dealing with -1
        if (bottom_im1[0] == -1):
            bottom_im1 = self.detect_yellow(image1)
        if (bottom_im2[0] == -1):
            bottom_im2 = self.detect_yellow(image2)
        if (top_im1[0] == -1):
            top_im1 = bottom_im1
        if (top_im2[0] == -1):
            top_im2 = bottom_im2
        
        x = top_im2[0] - bottom_im2[0]
        y = top_im1[0] - bottom_im1[0]
        z = top_im1[1] - bottom_im1[1]
        
        return np.array([x,y,z])
    
    
  def detect_joint_angles(self, image1, image2):
      v1 = self.get_link1_vector(image1, image2)
      v2 = self.get_link2_vector(image1, image2)
      v3 = self.get_link3_vector(image1, image2)
      
      # angle between links 2 and 3, we calculate the angle in 3d (not components), also have to indicate it's not joint2
      ja4 = self.angle_between(v2, v3, True, False)
      
      # angle between links 1 and 2, we don't use 3d as we need the component in xz plane, need to indicate that it's joint2 to use only x and z
      ja2 = self.angle_between(v1, v2, False, True)
      
      # angle between links 1 and 2, we don't use 3d as we need the component in yz plane, need to indicate that it's not joint2 to use only y and z
      ja3 = self.angle_between(v1, v2, False, False)
          
      return np.array([ja2, ja3, ja4])

  
  # Receive data from camera 1
  def callback1(self, data): 
      try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
            print(e)
      

  # Receive data from camera 2 and calls callback function
  def callback2(self, data): 
      try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
            print(e)
      
      self.callback(data)


  # Receive joint angles data and publish it
  def callback(self, data):
        image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
        #im = cv2.imshow('camera1 and camera 2', image)
        cv2.waitKey(1)
        
        angles = self.detect_joint_angles(self.cv_image1, self.cv_image2)

        self.joint2 = Float64()
        self.joint2.data = angles[0]
        
        self.joint3 = Float64()
        self.joint3.data = angles[1]
    
        self.joint4 = Float64()
        self.joint4.data = angles[2]
    
        try:
            self.joint2_pub.publish(self.joint2)
            self.joint3_pub.publish(self.joint3)
            self.joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)
      

# call the class
def main(args):
  vis1 = vision1()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
