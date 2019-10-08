import cv2
import numpy as np


class Feature:
    def __init__(self, shape, colour, centroid, contour):  # type: (str, str, np.ndarray, np.ndarray) -> None
        self.shape = shape
        self.colour = colour
        self.centroid = centroid
        self.contour = contour


class FeatureDetector:
    def __init__(self):
        pass

    def find_features(self, image):  # type: (np.ndarray) -> List[Feature]
        features = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = {
            'red': self.red_mask(hsv),
            'green': self.green_mask(hsv),
        }
        for col, mask in masks.items():
            cv2.imshow('{} mask'.format(col), mask.astype(np.uint8)*255)
            cv2.waitKey(3)
            _, contours, _ = cv2.findContours(
                mask.astype(np.uint8),
                mode=cv2.RETR_EXTERNAL,
                method=cv2.CHAIN_APPROX_SIMPLE,
            )
            for contour in contours:
                centroid = self.get_centroid(contour)
                shape = self.get_shape(contour)
                if shape is None:
                    continue
                features.append(Feature(shape, col, centroid, contour))
        return features

    @staticmethod
    def get_centroid(contour):
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return contour[0]
        return np.array([
            M['m10']/M['m00'],
            M['m01']/M['m00'],
        ])

    @staticmethod
    def get_shape(contour):
        if cv2.contourArea(contour) < 100.0:
            return None
        contour = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
        if len(contour) == 3:
            return 'triangle'
        elif len(contour) == 4:
            return 'square'
        elif len(contour) > 4:
            return 'circle'
        return None


    @staticmethod
    def red_mask(hsv):  # type: (np.ndarray) -> np.ndarray
        mask_low = cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 250])) > 0.0
        mask_high = cv2.inRange(hsv, np.asarray([170, 70, 50]), np.asarray([180, 255, 250])) > 0.0
        return mask_low | mask_high

    @staticmethod
    def green_mask(hsv):  # type: (np.ndarray) -> np.ndarray
        return cv2.inRange(hsv, np.asarray([40, 70, 50]), np.asarray([80, 255, 250])) > 0.0

    @staticmethod
    def col_name_to_rgb(colour):  # type: (str) -> Tuple[int, int, int]
        return {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
        }[colour]
