import rospy
from std_msgs.msg import Float64
from smach import State
import time


class StoppedState(State):
    def __init__(self, delay_time=.5):
        State.__init__(self, outcomes=['ok'])
        self.delay_time = delay_time
        self.rate = rospy.Rate(10)
        self.angle_error_pub = rospy.Publisher('angle_error', Float64, queue_size=1)
        self.linear_error_pub = rospy.Publisher('linear_error', Float64, queue_size=1)

    def execute(self, ud):
        start_time = time.time()
        while time.time() - start_time < self.delay_time:
            self.angle_error_pub.publish(0.)
            self.linear_error_pub.publish(0.)
            self.rate.sleep()

        return 'ok'
