#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy
import cv2
import rospkg
from cv_bridge import CvBridge, CvBridgeError


'''

rospy.init_node('image_pub', anonymous=True)


final = rospy.Publisher('final_image', Image, queue_size=10)

frame=cv2.imread('./src/img_to_laser/src/2.jpeg',0)

frame = numpy.asarray(frame)

msg_frame = CvBridge().cv2_to_imgmsg(frame)

image_pub.publish(msg_frame_edges, "mono8")

time.sleep(0.1)'''

'''
class ImagePublisher:

    def __init__(self):
        """ Constructor """
        self.image_pub = rospy.Publisher("final_image", Image,queue_size=200)
        self.cvbridge_obj = CvBridge()

    def img_pub_node(self):
        """ Publisher helper function """
        cv_img = cv2.imread('./src/img_to_laser/src/2.jpeg',0)

        # Convert this to ROS image format
        ros_img = self.cvbridge_obj.cv2_to_imgmsg(cv_img, encoding="mono8")
        try:
            self.image_pub.publish(ros_img)
            rate = rospy.Rate(0.1)
            rate.sleep()
        except CvBridgeError as bridgerr:
            print(bridgerr)

def main():
    "Main function"
    image_publisher_obj = ImagePublisher()
    rospy.init_node('img_pub_node', anonymous=True)
    image_publisher_obj.img_pub_node()
    rospy.spin()

if __name__ == '__main__':
    main()'''

def main():
    "Main function"
    rospy.init_node('image_pub', anonymous=True)
    final = rospy.Publisher('final_image', Image, queue_size=10)

    frame=cv2.imread('./src/img_to_laser/src/2.jpeg',0)

    frame = numpy.asarray(frame)

    msg_frame = CvBridge().cv2_to_imgmsg(frame)

    image_pub.publish(msg_frame_edges, "mono8")

    time.sleep(0.1)


if __name__ == '__main__':
    main()