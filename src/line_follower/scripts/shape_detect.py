#!/usr/bin/env python

import cv2
import cv_bridge
import rospy

from feature_detector import FeatureDetector


def main():
    rospy.init_node('shape_detect')
    bridge = cv_bridge.CvBridge()
    feature_detector = FeatureDetector()

    def image_callback(msg):
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        for feature in feature_detector.find_features(image):
            cv2.putText(
                image,
                '{} {}'.format(feature.colour, feature.shape),
                tuple(int(x) for x in feature.centroid),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 0),
            )
            col = FeatureDetector.col_name_to_rgb(feature.colour)
            cv2.drawContours(image, [feature.contour], -1, col, 5)

        cv2.imshow('image', image)
        cv2.waitKey(3)

    # rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    from glob import glob

    filenames = glob('images/**.jpg')

    while not rospy.is_shutdown():
        for filename in filenames:
            image = cv2.imread(filename)
            w = 640
            h = 640*image.shape[1]/image.shape[0]
            image = cv2.resize(image, (h, w))
            msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
            rate = rospy.Rate(10)
            start_time = rospy.get_time()
            while not rospy.is_shutdown():
                image_callback(msg)
                if rospy.get_time() - start_time > 2:
                    break
            rate.sleep()


if __name__ == '__main__':
    main()
