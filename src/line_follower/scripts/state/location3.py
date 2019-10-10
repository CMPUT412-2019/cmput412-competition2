from smach import State

import sys, os
sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), '../../../../')))

from src.line_follower.scripts.feature_detector import FeatureDetector, select_center
from src.line_follower.scripts.util import notify_match


class Location3State(State):
    def __init__(self, ud):
        State.__init__(self, outcomes=['ok'])
        self.feature_detector = FeatureDetector()
        self.ud = ud

    def execute(self, ud):
        features = self.feature_detector.get_features()
        feature = select_center(features)
        if feature.shape == self.ud.green_shape:
            notify_match()

        print(feature.shape)

        return 'ok'
