#!/usr/bin/env python  
# import rospy

# # Because of transformations
# import tf_conversions

# import tf2_ros
# import geometry_msgs.msg
# import turtlesim.msg


# def handle_turtle_pose(msg, turtlename):
#     br = tf2_ros.TransformBroadcaster()
#     t = geometry_msgs.msg.TransformStamped()

#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = "base_link"
#     t.child_frame_id = "optical_cam_center"
#     t.transform.translation.x = 0.0
#     t.transform.translation.y = 0.0
#     t.transform.translation.z = 0.0
#     q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
#     t.transform.rotation.x = q[0]
#     t.transform.rotation.y = q[1]
#     t.transform.rotation.z = q[2]
#     t.transform.rotation.w = q[3]

#     br.sendTransform(t)

# if __name__ == '__main__':
#     rospy.init_node('optical_cam_transform_publisher')
#     turtlename = rospy.get_param('~turtle')
#      rospy.Subscriber('/%s/pose' % turtlename,
#                       turtlesim.msg.Pose,
#                       handle_turtle_pose)
#     rospy.spin()
  
#import roslib
import rospy
from nav_msgs.msg import Odometry
import tf
#import turtlesim.msg

def func1(msg, name):
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0,0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),name,
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('optical_cam_transform_publisher')
    cam_frame="optical_cam_center"
    #odom=Odometry()
    rospy.Subscriber('/odom',
                     Odometry,
                     func1,cam_frame
                     )
    rospy.spin()
