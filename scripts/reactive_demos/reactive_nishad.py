#!/usr/bin/env python
<<<<<<< HEAD
=======
#Author: Nishad Gothoskar
>>>>>>> 70b05b347a0fc913abaec2e2ec6600532ebad03d
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lis_pr2_pkg.uber_controller import UberController

<<<<<<< HEAD
rospy.init_node("head_tracking")
=======
rospy.init_node("red_tracking")
>>>>>>> 70b05b347a0fc913abaec2e2ec6600532ebad03d

head_limits = np.array([[-2.0, -.5], [2.0, .5]])
dx = -0.06/320
dy = 0.04/240
eps = 0.03

class HeadListener:
	def __init__(self):
		self.bridge = CvBridge()
		self.UC = UberController()
		self.head_pos = np.array(self.UC.get_head_pose())
		self.sub = rospy.Subscriber('/head_mount_kinect/rgb/image_rect_color',
			Image,
			self.call_back,
			queue_size = 1)
<<<<<<< HEAD
		self.pub = rospy.Publisher('/head_tracking/face_detections', Image)
=======
>>>>>>> 70b05b347a0fc913abaec2e2ec6600532ebad03d
		
	def call_back(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
		except CvBridgeError as e:
			print(e)
		lower_green = np.array([100,0,0])
		upper_green = np.array([255,60,60])
		mask = cv2.inRange(cv_image, lower_green, upper_green)
<<<<<<< HEAD
		# with open ('/home/demo/Desktop/nishad_imgs/' +  str(self.counter) + ".txt","w+") as f:
		# 	f.write(str(mask))
		# cv2.imwrite('/home/demo/Desktop/nishad_imgs/' + str(self.counter) + ".png",mask)
		# cv2.imwrite('/home/demo/Desktop/nishad_imgs/' + str(self.counter) + "-normal.png",cv_image)
		# self.counter+=1
=======
>>>>>>> 70b05b347a0fc913abaec2e2ec6600532ebad03d

		detections = np.where(mask==255)[0]
		head_pos = self.head_pos
		image_shape = cv_image.shape

		if len(detections)>0:
			avg = np.mean(detections, axis=0)
			off_center = avg - np.array((image_shape[0]/2, image_shape[1]/2))
			if off_center[1] > eps*image_shape[1]:
				head_pos[1] += dy*np.abs(off_center[1])
			elif off_center[1] < - eps*image_shape[1]:
				head_pos[1] -= dy*np.abs(off_center[1])

		head_pos = np.clip(head_pos, *head_limits)
		self.head_pos = head_pos
		self.UC.command_head(head_pos, 0.2, False)

<<<<<<< HEAD
		try:
			self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

=======
>>>>>>> 70b05b347a0fc913abaec2e2ec6600532ebad03d
HeadListener()
rospy.spin()