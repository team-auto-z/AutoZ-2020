#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray,Point
from sensor_msgs.msg import Imu,NavSatFix


pub = rospy.Publisher("/imu/data",Imu,queue_size=10)
pub_gps = rospy.Publisher('/gps/fix',NavSatFix,queue_size=10)

def callback_imu(data):
    # rospy.loginfo(data.poses[])
    imu_data = Imu()
    imu_data.header.frame_id = "base_link"


    # poses[0].position-> ax, ay, az (linear acc)
    imu_data.linear_acceleration.x = data.poses[0].position.x
    imu_data.linear_acceleration.y = data.poses[0].position.y
    imu_data.linear_acceleration.z = data.poses[0].position.z
    #poses[0].orientation orientation quaternion
    imu_data.orientation.x = data.poses[0].orientation.x
    imu_data.orientation.y = data.poses[0].orientation.y
    imu_data.orientation.z = data.poses[0].orientation.z
    imu_data.orientation.w = data.poses[0].orientation.w
    #poses[1].position gx gy gz
    imu_data.angular_velocity.x = data.poses[1].position.x
    imu_data.angular_velocity.y = data.poses[1].position.y
    imu_data.angular_velocity.z = data.poses[1].position.z
    

    imu_data.angular_velocity_covariance = [1e-3,0,0,0,1e-3,0,0,0,1e-3]
    imu_data.linear_acceleration_covariance = [1e-3,0,0,0,1e-3,0,0,0,1e-3]
    imu_data.orientation_covariance = [1e-3,0,0,0,1e-3,0,0,0,1e-3]
    imu_data.header.stamp = rospy.Time.now()
    
    pub.publish(imu_data)

    # print(data.position, ' ', data.orientation)
    # print("Got Value")
    
def callback_gps(data):
    gps_data = NavSatFix()
    gps_data.header.frame_id = "map"
    gps_data.latitude  = data.x
    gps_data.longitude = data.y
    gps_data.position_covariance = [1e-3,0,0,0,1e-3,0,0,0,1e-3]
    gps_data.position_covariance_type = 3
    gps_data.header.stamp = rospy.Time.now()
    pub_gps.publish(gps_data)

def imuListener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('SensorSerialListener', anonymous=True)

    rospy.Subscriber("imuArray", PoseArray, callback_imu)
    rospy.Subscriber("gpsArray",Point,callback_gps)
    print("RUnning")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imuListener()