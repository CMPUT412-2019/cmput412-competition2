from state.linefollow import LineFollowState, TransitionAfter
from state.stopped import StoppedState

import rospy
import numpy as np
import cv2
from smach import StateMachine
from smach_ros import IntrospectionServer


forward_speed = 0.4


def white_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    return cv2.inRange(hsv, np.array([0, 0, 200]), np.array([255, 50, 255]))


def red_detector(hsv):   # type: (np.ndarray) -> np.ndarray
    return np.sum((
        cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 250])) |
        cv2.inRange(hsv, np.asarray([170, 70, 50]), np.asarray([180, 255, 250]))
    ).astype(bool)) > 300


rospy.init_node('competition2')


sm = StateMachine(outcomes=['ok', 'stop'])
with sm:
    StateMachine.add('FOLLOW', LineFollowState(forward_speed, white_filter, TransitionAfter(red_detector)), transitions={'ok': 'STOP'})
    StateMachine.add('STOP', StoppedState(), transitions={'ok': 'FOLLOW'})

sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
sis.start()

sm.execute()
