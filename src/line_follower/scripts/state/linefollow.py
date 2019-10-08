import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from smach import State
import cv2
import cv_bridge
import numpy as np


class StopBehavior:
    def init(self):
        pass

    def tick(self, hsv):
        pass


class FilterStopBehavior:
    def __init__(self, stop_filter):
        self.saw_stopline = False
        self.stop_filter = stop_filter

    def init(self):
        self.saw_stopline = False

    def tick(self, hsv):
        stop_mask = self.get_eye_mask(hsv) & self.stop_filter(hsv)

        stop_mask_empty = np.sum(stop_mask) < 300
        if self.saw_stopline and stop_mask_empty:
            return False
        else:
            self.saw_stopline = not stop_mask_empty

        cv2.imshow('stop_mask', stop_mask * 255)
        return True

    @staticmethod
    def get_eye_mask(hsv):
        h, w, d = hsv.shape
        search_top = int(1.5 * h / 4 - 20)
        search_bot = int(1.5 * h / 4)
        eye_mask = np.ones((h, w), dtype=bool)
        eye_mask[0:search_top, 0:w] = False
        eye_mask[search_bot:h, 0:w] = False
        return eye_mask


class LineFollowState(State):
    class StopError(Exception):
        pass

    def __init__(self, forward_speed, line_filter, stop_detector):
        State.__init__(self, outcomes=['ok', 'stop'])

        self.forward_speed = forward_speed
        self.line_filter = line_filter

        self.stop_detector = stop_detector  # type: StopBehavior

        self.bridge = cv_bridge.CvBridge()
        self.twist = Twist()
        self.image_sub = rospy.Subscriber('usb_cam_node/image_raw', Image, self.image_callback)
        self.angle_error_pub = rospy.Publisher('angle_error', Float64, queue_size=1)
        self.linear_error_pub = rospy.Publisher('linear_error', Float64, queue_size=1)

        self.rate = rospy.Rate(10)
        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def wait_for_image(self):
        while self.image is None:
            self.rate.sleep()

    def execute(self, ud):
        self.wait_for_image()
        self.stop_detector.init()

        try:
            while True:
                self.tick()
                self.rate.sleep()
        except self.StopError:
            return 'stop'

    def tick(self):
        image = np.copy(self.image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        eye_mask = self.get_eye_mask(hsv)
        line_mask = self.line_filter(hsv) & eye_mask

        # Check for stopping condition
        if not self.stop_detector.tick(hsv):
            raise self.StopError()

        # Line following
        if not self.is_mask_empty(line_mask):
            cx, cy = self.get_mask_center(line_mask)
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = (cx - image.shape[1] / 2) / float(image.shape[1])

            self.linear_error_pub.publish(-self.forward_speed * (1. - abs(err)))
            self.angle_error_pub.publish(err)

        # Display
        cv2.imshow('main_camera', image)
        cv2.imshow('line_mask',line_mask * 255)
        cv2.waitKey(3)

    @staticmethod
    def get_eye_mask(image):
        return np.ones(image.shape[:2]).astype('uint8')

    @staticmethod
    def get_mask_center(mask):
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy

    @staticmethod
    def is_mask_empty(mask):
        return np.all(mask == 0)