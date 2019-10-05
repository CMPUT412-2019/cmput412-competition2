#!/usr/bin/env python
from __future__ import division, print_function

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from util import SubscriberValue


class DetectLine:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.depth_image = SubscriberValue('/camera/depth_registered/image_raw', Image, transform=self.bridge.imgmsg_to_cv2)
        self.camera_model.fromCameraInfo(rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo))
        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.point_publisher = rospy.Publisher('/line_point', PointStamped, queue_size=1)
        self.depth_image.wait()

    def image_callback(self, image):  # type: (Image) -> None
        image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self.get_line_mask(image) & self.get_eye_mask(image)
        if np.sum(mask) > 0:
            cx, cy = self.get_mask_center(mask)
            d = self.depth_image.value[cy, cx]
            pos = np.array(self.camera_model.projectPixelTo3dRay((cx, cy))) * d
            print(pos)
            point = PointStamped()
            point.header.frame_id = self.camera_model.tfFrame()
            point.point.x = pos[0]
            point.point.y = pos[1]
            point.point.z = pos[2]
            self.point_publisher.publish(point)
        else:
            print('No line found')

    @staticmethod
    def get_mask_center(mask):
        M = cv2.moments(mask.astype(np.uint8))
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy

    @staticmethod
    def get_eye_mask(image):
        h, w, d = image.shape
        search_top = int(3 * h / 4)
        search_bot = h
        eye_mask = np.zeros((h, w), dtype=bool)
        eye_mask[search_top:search_bot, :] = True
        return eye_mask

    @staticmethod
    def get_line_mask(image):
        # return cv2.inRange(image, np.array([0,  0,  225]), np.array([255, 50, 255])) > 0.0
        return cv2.inRange(image, np.asarray([0, 0, 200]), np.asarray([179, 10, 250])) > 0.0


if __name__ == '__main__':
    rospy.init_node('detect_line')
    DetectLine()
    rospy.spin()
