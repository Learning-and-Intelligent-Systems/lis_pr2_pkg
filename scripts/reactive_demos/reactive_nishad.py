#!/usr/bin/env python
#Author: Nishad Gothoskar
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lis_pr2_pkg.uber_controller import UberController

rospy.init_node("red_tracking")

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
		
	def call_back(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
		except CvBridgeError as e:
			print(e)
		lower_green = np.array([100,0,0])
		upper_green = np.array([255,60,60])
		mask = cv2.inRange(cv_image, lower_green, upper_green)

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

HeadListener()
rospy.spin()