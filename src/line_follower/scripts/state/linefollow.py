import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from smach import State
import cv2
import cv_bridge
import numpy as np


class TransitionBehaviour:
    def __init__(self):
        pass

    def init(self):
        pass

    def tick(self, hsv):
        pass


class TransitionAfter(TransitionBehaviour):
    def __init__(self, func):
        TransitionBehaviour.__init__(self)

        self.func = func
        self.transitioning = False

    def init(self):
        self.transitioning = False

    def tick(self, hsv):
        transitioning = self.func(hsv)

        if self.transitioning and not transitioning:
            self.transitioning = False
            return False
        else:
            self.transitioning = transitioning
            return True


class LineFollowState(State):
    class StopError(Exception):
        pass

    def __init__(self, forward_speed, line_filter, transition_behaviour):
        State.__init__(self, outcomes=['ok'])

        self.forward_speed = forward_speed
        self.line_filter = line_filter

        self.transition_behaviour = transition_behaviour  # type: TransitionBehaviour

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
        self.transition_behaviour.init()

        try:
            while True:
                self.tick()
                self.rate.sleep()
        except self.StopError:
            return 'ok'

    def tick(self):
        image = np.copy(self.image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        eye_mask = self.get_eye_mask(hsv)
        line_mask = self.line_filter(hsv) & eye_mask

        # Check for stopping condition
        if not self.transition_behaviour.tick(hsv):
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