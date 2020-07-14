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
        def __init__(self,resize_width,resize_height):
            self.resize_width = resize_width
            self.resize_height = resize_height
            self.bridge = CvBridge()

            
            camera_info = rospy.wait_for_message('/usb_cam/camera_info', CameraInfo, timeout=5)
            # except(rospy.ROSException), e:
            #     print ("Camera info topic not available")
            #     print (e)
            #     exit()
            waf = float(resize_width) / camera_info.width
            haf = float(resize_height) / camera_info.height
            camera_info.height = resize_height
            camera_info.width = resize_width

            # adjust the camera matrix
            K = camera_info.K
            camera_info.K = (K[0]*waf,         0.,  K[2]*waf,
                                    0.,  K[4]*haf,  K[5]*haf,
                                    0.,        0.,         1.)

            # adjust the projection matrix
            P = camera_info.P
            camera_info.P = (P[0]*waf,        0.,  P[2]*waf,  0.,
                                0.,  P[5]*haf,  P[6]*haf,  0.,
                                0.,        0.,        1.,  0.)

            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(camera_info)
            print (camera_info)
        print(1)
            transform_listener = ros_tf.TransformListener()
            transform_listener.waitForTransform('/base_link', '/optical_cam_center', rospy.Time(0), rospy.Duration(5.0))
            cam_transform_translation, cam_transform_rotation = transform_listener.lookupTransform('/base_link', '/optical_cam_center', rospy.Time(0))
            self.cam_transform_rotation_matrix = ros_tf.transformations.quaternion_matrix(cam_transform_rotation)[:-1,:-1]
            self.cam_transform_translation = np.asarray(cam_transform_translation)
            print (self.cam_transform_translation)
            print (self.cam_transform_rotation_matrix)
