#!/usr/bin/env python

import cv2
import numpy as np 
from timeit import default_timer as timer

import rospy
import tf as ros_tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

class CNN:
    def __init__(self,):
    	print(3)        
	    start = timer()
        try:
             cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv_image = cv2.resize(cv_image, (self.resize_width, self.resize_height))
        im = np.asarray(cv_image)
        #image_pl = self.graph.get_tensor_by_name('Placeholder_1:0')
        #softmax = self.graph.get_tensor_by_name('Validation/decoder/Softmax:0')

        #output = self.sess.run([softmax], feed_dict={image_pl: im})
        shape = im.shape
        output_image = im
        # cv2.imshow('fig',im)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        threshold = 0.5
        im_threshold = output_image > threshold

        world_points = self.world_point_array[im_threshold]

        cv_output = np.uint8(255*im_threshold)
        #cv_output = cv2.cvtColor(im_threshold, cv2.COLOR_GRAY2BGR)
        msg_out = self.bridge.cv2_to_imgmsg(cv_output, 'bgr8')
        msg_out.header.stamp = image_msg.header.stamp
        #self.im_publisher.publish(msg_out)

        points = world_points
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = image_msg.header.stamp
        cloud_msg.header.frame_id = 'base_link'
        cloud_msg.height = 1
        cloud_msg.width = len(world_points)
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 4
        cloud_msg.row_step = 3 * len(world_points)
        cloud_msg.data = world_points.tostring()

        self.cloud_publisher.publish(cloud_msg)
        end = timer()
	print end - start
