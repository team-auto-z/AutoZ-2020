#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

def talker():
	#Topic
	#Message
	#Frequency
	#Pub Obj
	#Publish

	pub=rospy.Publisher('chatter',String,queue_size=10)

	rospy.init_node('talk',anonymous=True)
	rate=rospy.Rate(1)

	while not rospy.is_shutdown():
		hello="hello world"
		rospy.loginfo(hello)
		pub.publish(hello)
		rate.sleep()

if __name__ == '__main__':

	try:
		talker()
	except rospy.ROSInterruptException:
		pass
							
