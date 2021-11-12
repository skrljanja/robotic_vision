#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class vision1: 
    
    def __init__(self):
    
    	# defining subscribers
        self.image1_sub = rospy.Subscriber('camera1/robot/image_raw', Image, self.callback)
        self.image2_sub = rospy.Subscriber('camera2/robot/image_raw', Image, self.callback)
        
        # defining publishers  (btw are you sure we define each joint separately and not like in lab 1? "self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)")
        self.joint2 = rospy.Publisher('/joint_angle_2', Float64, queue_size = 10)
        self.joint3 = rospy.Publisher('/joint_angle_3', Float64, queue_size = 10)
        self.joint4 = rospy.Publisher('/joint_angle_4', Float64, queue_size = 10)
        
        # initialize the bridge between openCV and ROS  (not sure if we need this here)
    	self.bridge = CvBridge()
    	
    # we can rearrange the following code from lab 1 to complete part 2.1	
    	
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
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos)**2)
        return 3 / np.sqrt(dist)


    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self,image):
      a = self.pixel2meter(image)
      # Obtain the centre of each coloured blob 
      center = a * self.detect_yellow(image)
      circle1Pos = a * self.detect_blue(image) 
      circle2Pos = a * self.detect_green(image) 
      circle3Pos = a * self.detect_red(image)
      # Solve using trigonometry
      ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
      ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
      ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
      return np.array([ja1, ja2, ja3])
