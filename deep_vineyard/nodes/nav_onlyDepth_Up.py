#!/usr/bin/env python3
"""
navigation algorithm using only the depth algorithm
@author: Aghi Diego
"""

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from deep_vineyard.msg import tupla

import rospy
import numpy as np
import cv2


import threading



#working height and weight
w_depth=320
h_depth=240




class INIT():

	def __init__(self):
		rospy.init_node('ROS_API')
		global ranges, pose, angular_velocity, linear_acceleration, image, imgRGB,ang_vel_command 
		image = np.zeros((480, 640), np.uint8)
		ang_vel_command=0
		#imgRGB = np.zeros((299, 299, 3), np.uint8)
		
class OUTPUT():

	def __init__(self, cmd_vel='/cmd_vel'):
		self.pub_vel = rospy.Publisher(cmd_vel, Twist, queue_size=10)
		self.pub_tupla = rospy.Publisher('/control_value', tupla, queue_size=10)
		# rate = rospy.Rate(2)
		self.vel = Twist()
		self.tupla = tupla()

	def Move(self, linear_x, angular_theta):
		'''
		Publish the components of velocity
		:param linear_x: 
		:param angular_theta:
		:return: none
		'''
		self.vel.linear.x=linear_x
		self.vel.angular.z=angular_theta
		self.pub_vel.publish(self.vel)


	def Communication(self, flag, controller_data):
		'''
		Publish the results for the controllers
		:param flag_choice 0 DEPTH 1 ML: 
		:param distance or prediction:
		:return: none
		'''
		self.tupla.flag=flag
		self.tupla.control=controller_data
		self.pub_tupla.publish(self.tupla)
		


class INPUT(threading.Thread):

	def __init__(self, scan='/scan', odom='/odom', imu='/imu', camera='/camera/aligned_depth_to_color/image_raw'):
		threading.Thread.__init__(self)
		self.scan = scan
		self.odom = odom
		self.imu = imu
		self.camera = camera
		#self.cameraRGB = cameraRGB

	def run(self):
		self.sub_lid = rospy.Subscriber(self.scan, LaserScan, self.Lidar)
		self.sub_odom = rospy.Subscriber(self.odom, Odometry, self.Odometry)
		self.sub_odom = rospy.Subscriber(self.imu, Imu, self.Imu)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(self.camera, Image, self.Camera)
		#self.imageRGB_sub = rospy.Subscriber(self.cameraRGB, Image, self.CameraColor)
		rospy.spin()

	def Lidar(self, msg):
		'''
		Control the LiDAR inputs
		:param msg:
		:return: ranges of the 360 values as numpy array
		'''
		global ranges
		ranges = msg.ranges

	def Odometry(self, msg):
		'''
		Control the odometry from the robot
		:param msg:
		:return: pose [coordinates, euler angles] as numpy array
		'''
		global pose
		position = msg.pose.pose.position
		orientation = msg.pose.pose.orientation

		coordinates = [position.x, position.y, position.z]
		euler = list(trans.euler_from_quaternion(
			(orientation.x, orientation.y, orientation.z, orientation.w)
			))

		pose = [coordinates, euler] # np.asarray([coordinates, euler])
		# yaw = euler[2] * 180 / math.pi

	def Imu(self, msg):
		'''
		Control the Inertia Measurement Unit from the robot
		:param msg:
		:return: angular velocity and linear acceleration as numpy arrays
		'''
		global angular_velocity, linear_acceleration
		angular_velocity = np.asarray([msg.angular_velocity.x,
									   msg.angular_velocity.y,
									   msg.angular_velocity.z])
		linear_acceleration = np.asarray([msg.linear_acceleration.x,
										  msg.linear_acceleration.y,
										  msg.linear_acceleration.z])

	def Camera(self, data):
		global image
		image = cv2.resize(self.bridge.imgmsg_to_cv2(data, "16UC1"), (w_depth,h_depth)) #resizing image to reduce complexity and computational load

		


#to find the window where all the points are beyond a certain depth. Basically just thresholding the frame
def findWindow(frame):
	threshold=0.25
	r,col=frame.shape

	frame=np.where(frame>threshold, 1.0, 0)
	
	return frame



	

if __name__ == '__main__':


	areaThreshold=2000 #threshold of the areas to be considered. Area of point beyond a certain threshold/depth
	bridge=CvBridge()
	init = INIT()

	#IN = INPUT(camera="/camera/depth/image_rect_raw")
	IN = INPUT(camera="/camera/aligned_depth_to_color/image_raw")
	#IN = INPUT(camera="/camera/depth/image_raw")
	IN.start()


	OUT = OUTPUT()

	while not rospy.is_shutdown():
		
		try:
		   image = np.array(image, dtype=np.float32)   #img conversion
		   
		   cv2.normalize(image, image, 0, 1, cv2.NORM_MINMAX) #normalization 0 to 1
		   img_view_=image.copy() #just for visualization purposes

		   image=findWindow(image) #function to find the rectangle/window - the navigation goal
		   img_cast=image.astype("uint8") #casting of the img to do the needed following ops
		   
		   __,contours,hierarchy = cv2.findContours(img_cast, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #finding the contours
		   area=np.zeros(len(contours)) #init the vectors

		   for ind in range(len(contours)):     #for all the contours found, calculate the area
			   cnt=contours[ind]
			   area[ind] = cv2.contourArea(cnt)

		   index_max_rect= np.argmax(area)   #getting rid of the noise, just biggest area kept
		   cnt2=contours[index_max_rect]
		   if (area[index_max_rect]>areaThreshold):    #if area is greater than the threshold window/goal found     
				xx,yy,ww,hh = cv2.boundingRect(cnt2)    #def a rect around the found area
				
				cv2.rectangle(img_view_,(xx,yy),(xx+ww,yy+hh),(255,255,255),2)  #draw the rect
				
				OUT.Communication(0,int(xx+(ww/2)-((w_depth/2)-1)))  #sending command to the control module
	




		   
		   cv2.imshow('depth View', img_view_)

			
		  
		except Exception as e:
			print(e)


		
		k = cv2.waitKey(1) & 0xFF
		if k == 27:
			OUT.Move(0, 0)
			break
		
	cv2.destroyAllWindows()
	rospy.signal_shutdown('Closing the program...')
