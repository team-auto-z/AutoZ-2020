#!/usr/bin/env python

import rospy
from image_sensor_publisher import ImageConverter
from straight_lanes import road_lines
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
image_converter = ImageConverter()


x,y = None,None
pub = rospy.Publisher('/twist',Twist,queue_size=10)






def makeTwist(x,y,turn_dampening_factor=0.30,linear_dampening_factor=10):
	twist = Twist()

	twist.linear.x = y/linear_dampening_factor #* (x_max - x_min) + x_min
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = -x/turn_dampening_factor #* (r_max - r_min) + r_min


	if twist.linear.x > x_max:
 		twist.linear.x = x_max
 	if twist.linear.x < x_min:
 		twist.linear.x = x_min


 	if twist.angular.z > r_max:
 		twist.angular.z = r_max
 	if twist.angular.z < r_min:
		twist.angular.z = r_min

	return twist

# def shifted_tanh(x):
# 	return -0.2 *math.tanh(x)

def makeUnitVector(x,y):
	return x/math.sqrt(x**2+y**2),y/math.sqrt(x**2+y**2)


def callback(data):
	global x,y,pub
	frame = image_converter.convert_msg_to_image(data)
	x,y = road_lines(frame)
	x,y = makeUnitVector(x,y)
	twist = makeTwist(x,y)
	# print(twist)
	print(x,y)
	pub.publish(twist)


	# rospy.loginfo(rospy.get_caller_id() + "{},{}".format(x,y))



def main():
	global x_min,x_max,r_min,r_max
	
    
	rospy.init_node('LaneFollower',anonymous=True)
	rospy.Subscriber("/front_view", Image, callback)
	
	print("Running in LaneFollower Mode Press Z to discontue")
	


	W ,H= 100,100
	
	rate = rospy.Rate(rospy.get_param("~rate", 100))
	x_min = rospy.get_param("~x_min", -0.20)
	x_max = rospy.get_param("~x_max", 0.20)
	r_min = rospy.get_param("~r_min", -1.0)
	r_max = rospy.get_param("~r_max", 1.0)

	while not rospy.is_shutdown():
		# print(x,y)

		img = np.zeros((W,H))
		# print x,y
		if x!= None or y!=None:
			cv2.line(img,(W//2,H),(int(H*x)+W//2,int(H*y-H)),(255,255,255),1)
		cv2.imshow('frame',img)
		if cv2.waitKey(33) == ord('z'):
			break

		
		rate.sleep()
		
if __name__=='__main__':
	main()