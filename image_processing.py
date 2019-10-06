#!/usr/bin/env python
from plutodrone.msg import *
from pid_tune.msg import *
import rospy
import time
from std_msgs.msg import Float64
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32



class WayPoint():
	

	def __init__(self):
		print ("init me aaya")
		rospy.init_node('image', disable_signals = True)
		self.ros_bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
      		self.image=np.zeros((100,100,3),np.uint8)
		self.param1_red = [20, 20,55]
		self.param2_red = [50,50,115]
		self.lower_red = np.array(self.param1_red)    ## Convert the parameters into a form that OpenCV can understand
		self.upper_red = np.array(self.param2_red)
		self.param1_green= [ 10,50,10]
		self.param2_green= [ 40,100,40]
		self.lower_green = np.array(self.param1_green)    ## Convert the parameters into a form that OpenCV can understand
		self.upper_green = np.array(self.param2_green)
		self.param1_blue = [ 250,70,0]
		self.param2_blue = [255,140,70]
		self.lower_blue = np.array(self.param1_blue)    ## Convert the parameters into a form that OpenCV can understand
		self.upper_blue = np.array(self.param2_blue)  

	def callback(self,msg):
		print ("callback me aaya")
		self.frame = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		self.image = self.frame[ 30:410, 160:530]
  	
	def detect_red(self):
		print ("d_r me aaya")
		self.mask  = cv2.inRange(self.image, self.lower_red, self.upper_red)
		self.res   = cv2.bitwise_and(self.image, self.image, mask= self.mask)
		_ , self.contours, _ = cv2.findContours(self.mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		if len(self.contours) != 0:
			# draw in red the contours that were founded
			#find the biggest area
			self.c = max(self.contours, key = cv2.contourArea)
			self.x,self.y,self.w,self.h = cv2.boundingRect(self.c)
			 # draw the book contour (in green)
			cv2.rectangle(self.image,(self.x,self.y),(self.x+self.w,self.y+self.h),(0,0,255),2)
		#red.publish(Float64(len(contours)))
		cv2.imshow('image',self.image)
 

  
	def image_process(self):
		print ("ip me aaya")
		self.detect_red()
		#self.detect_green()
		#self.detect_red()



if __name__ == '__main__':
	while not rospy.is_shutdown():
		print ("main me aaya")
		temp = WayPoint()
		temp.image_process()
		rospy.spin()
