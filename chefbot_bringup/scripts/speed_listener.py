#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import Float32
import serial
class SpeedListener:
	def __init__(self,sending_speed_to_motor_driver=False):
		self.lmotor_cmd=0
		self.rmotor_cmd=0
		if sending_speed_to_motor_driver:
			self.s = serial.Serial('/dev/ttyACM0',9600)

	def send_serially(self):
		##print("Sending send_serially")
		rospy.loginfo(rospy.get_caller_id() + "L:{} R:{}".format(speed_listener.lmotor_cmd,speed_listener.rmotor_cmd))
		rospy.loginfo("SENDING DATA SERIALLY")
		print("{} {}".format(int(self.lmotor_cmd),int(self.rmotor_cmd)).encode())
		self.s.write("{} {}".format(int(self.lmotor_cmd),int(self.rmotor_cmd)).encode())
		time.sleep(0.05)


	#def read_serial_callback(self):
	#	rospy.loginfo("\t\t\n\n"+self.s.readline())
		

speed_listener = SpeedListener(True)


def callback_right_motor(data):
	speed_listener.rmotor_cmd=data.data
 	# rospy.loginfo(rospy.get_caller_id() + " L:{} R:{}".format(speed_listener.lmotor_cmd,speed_listener.rmotor_cmd))
 	# speed_listener.send_serially()nan

def callback_left_motor(data):
	speed_listener.lmotor_cmd=data.data
	# rospy.loginfo(rospy.get_caller_id() + "L:{} R:{}".format(speed_listener.lmotor_cmd,speed_listener.rmotor_cmd))
	# speed_listener.send_serially()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('MotorSerialCmd', anonymous=True)
    rate = rospy.Rate(rospy.get_param("~rate", 50))
    rospy.Subscriber("/lmotor_cmd", Float32, callback_right_motor)
    rospy.Subscriber("/rmotor_cmd",Float32,callback_left_motor)



    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
    	speed_listener.send_serially()
	#speed_listener.read_serial_callback()
    	rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    listener()
