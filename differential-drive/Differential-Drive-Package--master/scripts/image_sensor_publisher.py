#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge,CvBridgeError
import matplotlib.pyplot as plt

class ImageConverter:
	def __init__(self):
		self.image = None
		self.converted_data = None
		self.bridge = CvBridge()
	def convert_image_to_msg(self,image):
		self.image = image
		try:
			self.converted_data = self.bridge.cv2_to_imgmsg(image,"bgr8")
		except CvBridgeError as e:
			print(e)

		return self.converted_data

	def convert_msg_to_image(self,data):
		self.data = data
		try:
			self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		return self.image






def publisher():

	rospy.init_node('CameraSensorNode',anonymous=False)
	pub = rospy.Publisher('/front_view', Image, queue_size=10)
	rate = rospy.Rate(rospy.get_param("~rate", 50))
	cap = cv2.VideoCapture(0)

	image_converter = ImageConverter()
	FPS = cap.get(cv2.CAP_PROP_FPS)

	
	# plt.figure()
	# plt.ion()
	while not rospy.is_shutdown():
		
		ret,frame = cap.read()
		data = image_converter.convert_image_to_msg(frame)
		# cv2.imshow('frame',frame)
		# if cv2.waitKey(24) & 0xFF == ord('q'):
		# 	break
		pub.publish(data)
		try:
			pass
			
		except KeyboardInterrupt:
			cv2.destroyAllWindows()
			break

if __name__ == '__main__':
	publisher()
