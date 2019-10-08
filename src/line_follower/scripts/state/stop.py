import rospy
from smach import State
import time


class StopState(State):
    def __init__(self, delay_time=.5):
        State.__init__(self, outcomes=['ok'])
        self.delay_time = delay_time
        self.rate = rospy.Rate(10)

    def execute(self, ud):
        start_time = time.time()
        while time.time() - start_time < self.delay_time:
            self.rate.sleep()

        return 'ok'
