from state.linefollow import LineFollowState, TransitionAfter, TransitionAt
from state.stop import StopState
from state.rotate import RotateState
from pid_control import PIDController

import rospy
import numpy as np
import cv2
from smach import StateMachine
from smach_ros import IntrospectionServer


forward_speed = 0.4
kp = 4.
ki = 0.
kd = 0.


def white_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    return cv2.inRange(hsv, np.array([0, 0, 200]), np.array([255, 50, 255])).astype(bool)


def red_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    return (
            cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 250])) |
            cv2.inRange(hsv, np.asarray([170, 70, 50]), np.asarray([180, 255, 250]))
    ).astype(bool)


def red_detector(hsv):   # type: (np.ndarray) -> np.ndarray
    return np.sum(red_filter(hsv)) > 300


rospy.init_node('competition2')


sm = StateMachine(outcomes=['ok', 'stop'])
with sm:
    StateMachine.add('FOLLOW1', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(red_detector)), transitions={'ok': 'FORWARD1'})
    StateMachine.add('FORWARD1', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter, TransitionAfter(red_detector)), transitions={'ok': 'TURN1'})
    StateMachine.add('TURN1', RotateState(), transitions={'ok': 'FOLLOW2'})
    StateMachine.add('FOLLOW2', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(red_detector)), transitions={'ok': 'TURN2'})
    StateMachine.add('TURN2', RotateState(np.pi), transitions={'ok': 'FOLLOW3'})
    StateMachine.add('FOLLOW3', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(red_detector)), transitions={'ok': 'FORWARD2'})
    StateMachine.add('FORWARD2', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter, TransitionAfter(red_detector)), transitions={'ok': 'TURN3'})
    StateMachine.add('TURN3', RotateState(), transitions={'ok': 'FOLLOW4'})
    StateMachine.add('FOLLOW4', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(red_detector)))

sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
sis.start()

sm.execute()
