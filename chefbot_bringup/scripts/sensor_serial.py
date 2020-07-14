#! /usr/bin/env python
import time
import rospy
import serial
from std_msgs.msg import Int16
class SensorSerial:
	def __init__(self):
		self.lmotor_enc=0
		self.rmotor_enc=0
		self.s = serial.Serial('/dev/ttyACM1',9600)
		self.sensor_string = ""
		

	def recv_serially(self):
		##print("Sending send_serially")
		
		self.sensor_string = self.s.readline()
		print(self.sensor_string)
		self.lmotor_enc,self.rmotor_enc = int(self.sensor_string.split(' ')[0]),int(self.sensor_string.split(' ')[1])
		rospy.loginfo(rospy.get_caller_id() + "L:{} R:{}".format(speed_listener.lmotor_enc,speed_listener.rmotor_enc))
		time.sleep(0.05)


	#def read_serial_callback(self):
	#	rospy.loginfo("\t\t\n\n"+self.s.readline())
		


speed_listener = SensorSerial()

   
def publisher():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub_r = rospy.Publisher('/rwheel', Int16, queue_size=10)
    pub_l = rospy.Publisher('/lwheel', Int16, queue_size=10)
    rospy.init_node('SensorSerialNode', anonymous=True)
    rate = rospy.Rate(50)
    


    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
    	speed_listener.recv_serially()
    	pub_l.publish(speed_listener.lmotor_enc)
    	pub_r.publish(speed_listener.rmotor_enc)
	#speed_listener.read_serial_callback()
    	rate.sleep()

    # rospy.spin()

if __name__ == '__main__':
    publisher()
