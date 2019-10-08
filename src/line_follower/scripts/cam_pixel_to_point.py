#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from line_follower.srv import CamPixelToPoint, CamPixelToPointResponse
from sensor_msgs.msg import CameraInfo, Image

from util import SubscriberValue


class CamPixelToPointServer:
    def __init__(self):
        self.camera_model = PinholeCameraModel()
        self.bridge = CvBridge()
        in_simulator = rospy.get_param('~in_simulator')
        depth_topic = '/camera/depth/image_raw' if in_simulator else '/camera/depth_registered/image_raw'
        self.camera_model.fromCameraInfo(SubscriberValue('/camera/rgb/camera_info', CameraInfo).value)
        self.depth_image = SubscriberValue(depth_topic, Image, transform=self.bridge.imgmsg_to_cv2)
        self.service = rospy.Service('cam_pixel_to_point', CamPixelToPoint, self.handle_service)
        print('Service is ready.')

    def handle_service(self, req):  # type: (CamPixelToPoint) -> CamPixelToPointResponse
        x, y = int(req.cam_pixel.x), int(req.cam_pixel.y)
        d = self.depth_image.value[y, x]
        pos = np.array(self.camera_model.projectPixelTo3dRay((x, y))) * d
        point = PointStamped()
        point.header.frame_id = self.camera_model.tfFrame()
        point.point.x, point.point.y, point.point.z = pos[0], pos[1], pos[2]
        return CamPixelToPointResponse(point)


if __name__ == '__main__':
    rospy.init_node('cam_pixel_to_point')
    CamPixelToPointServer()
    rospy.spin()
