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


class vision2:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    
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
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    	
  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
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


  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      # take green and red as these are the edge circles
      circle1Pos = self.detect_red(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      # 10 because that is the distance between the circles in meters (4+0+3.2+2.8)
      return 10 / np.sqrt(dist)


  # Calculate the relevant joint angles from the image
  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_green(image)
    circle1Pos = a * self.detect_yellow(image) 
    circle2Pos = a * self.detect_blue(image) 
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    # asssume that joint 2 is fixed
    # ja2 = 0
    # distance between joints 2 and 3 is 0  
    ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja3 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja4 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja1, ja3, ja4])
  

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    a = self.detect_joint_angles(self.cv_image1)
    #im1=cv2.imshow('window1', self.cv_image1)
    #cv2.waitKey(1)
    
    # assigning angle values
    self.joint1 = Float64()
    self.joint1.data = a[0]
    
    self.joint3 = Float64()
    self.joint3.data = a[1]
    
    self.joint4 = Float64()
    self.joint4.data = a[2]
    
    # self.joints = Float64MultiArray()
    # self.joints.data = a
    
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      # self.joints_pub.publish(self.joints)
      self.joint1_pub.publish(self.joint1)
      self.joint3_pub.publish(self.joint3)
      self.joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)
      
  # Recieve data from camera 2, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      #self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    
    #a = self.detect_joint_angles(self.cv_image1)
    a = self.detect_joint_angles(self.cv_image2)
    
    image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
    im = cv2.imshow('camera1 and camera 2', image)
    #im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)
    
    # assigning angle values
    self.joint1 = Float64()
    self.joint1.data = a[0]
    
    self.joint3 = Float64()
    self.joint3.data = a[1]
    
    self.joint4 = Float64()
    self.joint4.data = a[2]
    #self.joints = Float64MultiArray()
    #self.joints.data = a

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      #self.joints_pub.publish(self.joints)
      self.joint1_pub.publish(self.joint1)
      self.joint3_pub.publish(self.joint3)
      self.joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)
      

# call the class
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
    
