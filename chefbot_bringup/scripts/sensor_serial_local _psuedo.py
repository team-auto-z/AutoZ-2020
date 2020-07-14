#!/usr/bin/env python


import serial
import rospy
from sensor_msgs.msg import Imu,NavSatFix
def main():
	
	rospy.init_node('SensorLocalisationReciever',anonymous=True)
	
	
	print("Running SensorLocalisationReciever")
	# ser = serial.Serial(rospy.get_param('~sensorserialport','/dev/ttyACM0'))

	
	imu_data = Imu()

	gps_data = NavSatFix()
	

	imu_data.header.frame_id = 'base_link'
	gps_data.header.frame_id = 'map'

	rate = rospy.Rate(rospy.get_param("~rate", 100))

	imu_pub = rospy.Publisher('/imu/data',Imu,queue_size=10)
	gps_pub = rospy.Publisher('/gps/fix',NavSatFix,queue_size=10)
	
	while not rospy.is_shutdown():
		# print(x,y)


		# data = ser.readline()
		
		# if data[0]=='#':
		# data = data[1:]
		# linear,angular = data.split('/')[0],data.split('/')[1]

		linear = [float(x) for x in linear.split(' ')]
		angular = [float(x) for x in angular.split(' ')]
		
		imu_data.orientation.x = 1 ##angular[3]
		imu_data.orientation.y = 1 ## angular[4]
		imu_data.orientation.z = 1 ##angular[5]

		imu_data.linear_acceleration.x = 1# linear[6]
		imu_data.linear_acceleration.y = 1# linear[7]
		imu_data.linear_acceleration.z = 1#linear[8]

		imu_data.angular_velocity_covariance = [1.0,0,0,0,1.0,0,0,0,1.0]
		imu_data.linear_acceleration_covariance = [1.0,0,0,0,1.0,0,0,0,1.0]


		gps_data.latitude  =1# linear[0]
		gps_data.longitude = 1#linear[1]
		gps_data.position_covariance = [1.0,0,0,0,1.0,0,0,0,1.0]
		gps_data.position_covariance_type = 3
		gps_data.header.stamp = imu_data.header.stamp = rospy.Time.now()
		
		imu_pub.publish(imu_data)
		gps_pub.publish(gps_data)
		print(linear)

			
			# print(angular)


			
		rate.sleep()
		
if __name__=='__main__':
	main()