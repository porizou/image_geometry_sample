#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import tf2
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs


class Sample:
    def __init__(self):
        self.pinhole = image_geometry.PinholeCameraModel()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransofrmListener(self.tfBuffer)

        self.bridge = CvBridge()

        depth = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image', Image)
        image = message_filters.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image)
        ts = message_filters.ApproximateTimeSynchronizer([depth , image], 100 , 0.3)

        ts.registerCallback(self.calc_distance)

        cam_info = rospy.wait_for_message('/hsrb/head_rgbd_sesor/rgb/camera_info', CameraInfo)
        self.pinhole.fromCameraInfo(cam_info)


    def calc_distance(self, depth_data, image_data):
        depth = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')
        img = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

        #TODO:
        y = 10
        x = 10

        ray = self.pinhole.projectPixelTo3dRay([x, y])
        camera_point = np.array(ray) * depth[y, x]

        cam_point = PointStamped()
        cam_point.header.frame_id = image_data.frame_id
        cam_point.point.x = camera_point[0]
        cam_point.point.y = camera_point[1]
        cam_point.point.z = camera_point[2]

        base_stamp = self.tfBuffer.transform(cam_point, "base_footprint", timeout=rospy.Duration(5))


if __name__ == "__main__":
    rospy.init_node("calc_obj_distance")
    sample = Sample()
    rospy.spin()
    
