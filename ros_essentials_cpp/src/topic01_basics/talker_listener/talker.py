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
	rospy.Rate(10)

	while not rospy.is_shutdown():
		hello='hello world' % rospy.get_time().to_sec()
		#rospy.loginfo(hello)
		pub.publish(hello)
		rate.sleep()
if __name__ == '__main__':

	try:
		talker()
	except rospy.ROSInterruptExecution:
		pass
							