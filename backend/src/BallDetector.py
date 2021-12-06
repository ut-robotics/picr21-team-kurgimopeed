import cv2
import numpy as np
from math import sin, cos, pi, sqrt
from src.Tools import calculateCoordinate, linear_map
from src.RSCamera import camera_angle, camera_fov, camera_transformation
from threading import Thread

class BallDetector():
    def __init__(self):
        self.threshold_lower = np.array([0, 0, 0])
        self.threshold_upper = np.array([0, 0, 0])

        blobparams = cv2.SimpleBlobDetector_Params()

        blobparams.filterByColor = True
        blobparams.blobColor = 255

        #blobparams.filterByArea = True
        blobparams.minArea = 25
        blobparams.maxArea = 100000

        #blobparams.minDistBetweenBlobs = 100

        blobparams.filterByCircularity = True
        blobparams.minCircularity = 0.1
        blobparams.maxCircularity = 1

        blobparams.filterByConvexity = True
        blobparams.minConvexity = 0.5

        blobparams.filterByInertia = False
        #blobparams.minInertiaRatio = 0.01

        self.detector = cv2.SimpleBlobDetector_create(blobparams)

        self.debug_mask = None
        #self.process_queue = Queue()
        self.locations = []

    def set_threshold(self, threshold_values):
        lower = threshold_values["balls"][:3]
        upper = threshold_values["balls"][3:]
        self.threshold_lower = np.array([lower])
        self.threshold_upper = np.array([upper])

    def get_mask(self, hsv_frame):
        mask = cv2.inRange(hsv_frame, self.threshold_lower, self.threshold_upper)

        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def get_debug_mask(self):
        return self.debug_mask
        
    # should ball tracking be done here?
    def getLocations(self, hsv_frame, depth_frame):
        mask = self.get_mask(hsv_frame)

        debug_mask = np.zeros(mask.shape, dtype=np.uint8)

        keypoints = self.detector.detect(mask)

        locations = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            radius = keypoint.size/2

            ball_mask = np.zeros(mask.shape, dtype=np.uint8)
            ball_mask = cv2.circle(ball_mask, (int(x),int(y)),int(radius), (255,255,255),cv2.FILLED)

            filtered_depth = cv2.bitwise_and(depth_frame, depth_frame, mask=ball_mask)

            measure_point_count = np.count_nonzero(filtered_depth)
            if measure_point_count > 0:
                dist = np.sum(filtered_depth)/measure_point_count
                debug_mask = cv2.bitwise_or(debug_mask, ball_mask)

                dist /= 1000 #mm to m

                locations.append(calculateCoordinate(x, y, dist, mask.shape))

        self.locations = locations
        self.debug_mask = debug_mask
        return locations
