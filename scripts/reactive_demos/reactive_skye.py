#!/usr/bin/env python
#Author: Skye Thompson
import rospy
import os
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from lis_pr2_pkg.uber_controller import Uber
import pdb


rospy.init_node("blob_det", anonymous=True)

class Detector():
	def __init__(self):
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("/head_mount_kinect/rgb/image_rect_color", Image, self.cb, queue_size=1)
		self.pub = rospy.Publisher("blob_loc", String, queue_size=1)

		self.uc = Uber()
		rospy.loginfo("Detector initialized.")
	def cb(self, data):
		#Detect green box!
		params = cv2.SimpleBlobDetector_Params()
		params.filterByArea = True
		params.filterByCircularity = False
		params.filterByConvexity = False
		params.minArea = 150
		params.maxArea = 10e20
		lower_blue = np.array([40,100,100])
		upper_blue = np.array([80,255,255])
		image_cv = self.bridge.imgmsg_to_cv2(data)
		blur = cv2.GaussianBlur(image_cv,(41,41),0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		im = cv2.inRange(hsv, lower_blue, upper_blue)
		im = cv2.bitwise_not(im)
		detect = cv2.SimpleBlobDetector(params)
		keypoints = detect.detect(im)
		im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

		#Head tracks green box, keeping it at coordinates [300, 300]
		self.uc.command_head([clamp_head((300 - keypoints[0].pt[0]) / 100), clamp_head(-(300 - keypoints[0].pt[1]) / 100)], time=1, blocking=False)
		self.pub.publish(str((300 - keypoints[0].pt[0]) / 100) + " " + str((300 - keypoints[0].pt[1]) / 100))
	def clamp_head(n):
		return max(-1.5, min(n, 1.5)) #So the head doesn't swing too far

uc = Uber()
uc.command_joint_pose('l', [0]*7, time=2, blocking=True)
uc.command_joint_pose('r', [0]*7, time=2, blocking=True)
uc.look_down_center()

det = Detector()
rospy.spin()