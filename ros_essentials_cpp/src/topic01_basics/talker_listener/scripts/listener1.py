#!/usr/bin/env python

import rospy
from std_msgs.msg import String 

def callback(message):
	rospy.loginfo(rospy.get_caller_id()+"I heard %s",message.data)
def listen():
	#topic
	#message
	#callback
	#check message
	#spin
	rospy.init_node('lis',anonymous=True)
	rospy.Subscriber('chatter',String,callback)
	rospy.spin()
if __name__ == '__main__':
	listen()
