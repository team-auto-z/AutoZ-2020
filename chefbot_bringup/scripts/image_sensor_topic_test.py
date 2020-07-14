#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from image_sensor_publisher import ImageConverter
import time
import numpy as np
##from speed_listener import SpeedListener
from std_msgs.msg import Float32


class SpeedListener:
	def __init__(self,sending_speed_to_motor_driver=False):
		self.lmotor_cmd=0
		self.rmotor_cmd=0
		


image_converter = ImageConverter()

image_list = []
speed_list = []
RECORD = True
NAME_VIDEO = '{}-{}'.format('Road',time.time())
PATH_VIDEO = './dataset/{}.mp4'.format(NAME_VIDEO)
HEIGHT,WIDTH = 640,480 
FPS = 25

speed_listener = SpeedListener()

def callback_right_motor(data):
	speed_listener.rmotor_cmd=data.data
 	# rospy.loginfo(rospy.get_caller_id() + " L:{} R:{}".format(speed_listener.lmotor_cmd,speed_listener.rmotor_cmd))
 	# speed_listener.send_serially()

def callback_left_motor(data):
	speed_listener.lmotor_cmd=data.data
	# rospy.loginfo(rospy.get_caller_id() + "L:{} R:{}".format(speed_listener.lmotor_cmd,speed_listener.rmotor_cmd))
	# speed_listener.send_serially()
    


def callback_image(data):
	
	frame = image_converter.convert_msg_to_image(data)
	if RECORD:

		image_list.append(frame)
		speed_list.append([speed_listener.lmotor_cmd,speed_listener.rmotor_cmd])


	cv2.imshow('frame',frame)
	cv2.waitKey(24)
	print(len(image_list))
	rospy.loginfo(rospy.get_caller_id() + "Recieved Image MSG")




def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('DataCollector', anonymous=True)
    rate = rospy.Rate(rospy.get_param("~rate", 10))
    rospy.Subscriber("/front_view", Image, callback_image)
    rospy.Subscriber("/lmotor_cmd", Float32, callback_right_motor)
    rospy.Subscriber("/rmotor_cmd",Float32,callback_left_motor)


    out = cv2.VideoWriter(PATH_VIDEO,cv2.VideoWriter_fourcc(*'DIVX'),FPS,(HEIGHT,WIDTH))



    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
	
		
		
    	try:
    		rate.sleep()

    		
    	except KeyboardInterrupt:
			cv2.destroyAllWindows()
			# print("Saving Video .......")
			break

    if RECORD==True:

	    for i in range(len(image_list)):
	    	print('#'*i)
	    	out.write(image_list[i])
	    	print(image_list[i].shape)
	    print("Saving Video .......")
	    np.save('./dataset/{}.npy'.format(NAME_VIDEO),np.array(speed_list))
    out.release()

    
if __name__ == '__main__':
    listener()

