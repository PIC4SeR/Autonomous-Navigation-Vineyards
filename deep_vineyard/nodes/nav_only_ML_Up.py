#!/usr/bin/env python
"""
navigation using only machine learning model

@author: Aghi Diego
"""

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as IMM
from keras.models import load_model 
from deep_vineyard.msg import tupla

import rospy
import numpy as np
import cv2, os


import threading
import tf.transformations as trans

class INIT():

	def __init__(self):
		rospy.init_node('ROS_API')
		global ranges, pose, angular_velocity, linear_acceleration, image, imgRGB
		image = np.zeros((480, 640), np.uint8)
		imgRGB = np.zeros((224, 224, 3), np.uint8)
		#224 224 for MobileNet , 299 299 for Xception
class OUTPUT():

	def __init__(self, cmd_vel='/cmd_vel'):
		self.pub_vel = rospy.Publisher(cmd_vel, Twist, queue_size=10)
		self.pub_tupla = rospy.Publisher('control_value', tupla, queue_size=10)
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

	def __init__(self, scan='/scan', odom='/odom', imu='/imu', camera='/camera/aligned_depth_to_color/image_raw', cameraRGB='/camera/color/image_raw'):
		threading.Thread.__init__(self)
		self.scan = scan
		self.odom = odom
		self.imu = imu
		#self.camera = camera
		self.cameraRGB = cameraRGB

	def run(self):
		self.sub_lid = rospy.Subscriber(self.scan, LaserScan, self.Lidar)
		self.sub_odom = rospy.Subscriber(self.odom, Odometry, self.Odometry)
		self.sub_odom = rospy.Subscriber(self.imu, Imu, self.Imu)
		self.bridge = CvBridge()
		#self.image_sub = rospy.Subscriber(self.camera, Image, self.Camera)
		self.imageRGB_sub = rospy.Subscriber(self.cameraRGB, Image, self.CameraColor)
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


	def CameraColor(self, data):
		global imgRGB
		#imgRGB = self.bridge.imgmsg_to_cv2(data, "bgr8")
		imgRGB = cv2.resize(self.bridge.imgmsg_to_cv2(data, "bgr8"), (224,224)) #fare il resize
		
#load the desired ML model
def deepVineyardModel(pathModel):
	model = load_model(pathModel)
	return model

if __name__ == '__main__':
	
	#model path definition
	real_path = os.path.dirname(os.path.realpath(__file__))
	model=deepVineyardModel(os.path.join(real_path,'models','MobileNet_final_retrained.h5'))
	#pre defined classes/labels
	classes = ['left', 'center', 'right']


	bridge=CvBridge()
	init = INIT()

	IN2 = INPUT(cameraRGB="/camera/color/image_raw")
	IN2.start()


	OUT = OUTPUT()


	#fourcc = cv2.VideoWriter_fourcc(*'XVID')
	#out = cv2.VideoWriter('output60.avi',fourcc,60.0,(640,480))
	while not rospy.is_shutdown():
		#im = IMM.fromarray(imgRGB)
		#im.save('vitto/prova.jpg')
		try:
		   #imgRGB = np.array(imgRGB, dtype=np.uint8)

		   #gigio=imgRGB.copy() #just to see the img at the end
		   #scale the img
		   imgRGB=(imgRGB/255.)
		   y_pred = model.predict(imgRGB[None,...]) #adding one dimension
		   ML_predict=np.argmax(y_pred, axis=-1)[0] #reading model prediction
		   #OUT.Communication(1,ML_predict)   #SENDING COMMANDS
		   #print(y_pred)
		   print(classes[ML_predict])
		   #cv2.imshow('ROS API Controller', intermedio)
		   #out.write(imgRGB)

		   #constant control=  linear 0.8 /  angular 0.3 !riguardare
		   #if ML_predict == 0:     #right
		   #		OUT.Move(0.2,-0.3)
		   #elif ML_predict == 1:   #center
		   #		OUT.Move(0.8, 0) 
		   #elif ML_predict == 2:    #left
		   #		OUT.Move(0.2,0.3)
		   #else:
		   #		OUT.Move(0,0)
		   #		print("Error ML-controller")


		   cv2.imshow('ROS API color', imgRGB)
		  
		except Exception as e:
			print(e)

		
		k = cv2.waitKey(1) & 0xFF
		if k == 27:
			OUT.Move(0, 0)
			break
	
	#out.release()
	cv2.destroyAllWindows()
	rospy.signal_shutdown('Closing the program...')
