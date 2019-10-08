import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from smach import State, StateMachine
from smach_ros import IntrospectionServer
import cv2
import cv_bridge
import numpy as np
import time

from state.linefollow import LineFollowState, FilterStopBehavior
from state.stop import StoppedState


def make_realcourse_follow_state():
    forward_speed = 0.4
    line_filter = lambda hsv: cv2.inRange(hsv, np.array([0,  0,  200]), np.array([255, 50, 255]))
    stop_filter = (lambda hsv: cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 250])) |
                                cv2.inRange(hsv, np.asarray([170, 70, 50]), np.asarray([180, 255, 250])))

    return LineFollowState(
        forward_speed=forward_speed,
        line_filter=line_filter,
        stop_detector=FilterStopBehavior(stop_filter=stop_filter)
    )


rospy.init_node('follower')

sm = StateMachine(outcomes=['ok', 'stop'])
with sm:
    StateMachine.add('FOLLOW', make_realcourse_follow_state(), transitions={'stop': 'STOP'})
    StateMachine.add('STOP', StoppedState(), transitions={'ok': 'FOLLOW'})

sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
sis.start()

sm.execute()
