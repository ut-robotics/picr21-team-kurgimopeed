import cv2
import numpy as np
from src.Tools import calculateCoordinate, depth_distance

class BallDetector():
    def __init__(self):
        self.threshold_lower = np.array([0, 0, 0])
        self.threshold_upper = np.array([0, 0, 0])

        blobparams = cv2.SimpleBlobDetector_Params()

        blobparams.filterByColor = True
        blobparams.blobColor = 255

        #blobparams.filterByArea = True
        blobparams.minArea = 10
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
        self.locations_on_frame = []

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
        self.locations_on_frame = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            radius = keypoint.size/2

            min_x, max_x = int(x-radius), int(x+radius)
            min_y, max_y = int(y-radius), int(y+radius)
            if min_x < 0:
                min_x = 0
            if max_x > depth_frame.shape[1]-1:
                max_x = depth_frame.shape[1]-1
            if min_y < 0:
                min_y = 0
            if max_y > depth_frame.shape[0]-1:
                max_y = depth_frame.shape[0]-1
        
            section_depth_frame = depth_frame[min_y:max_y, min_x:max_x]
            shape = (max_y-min_y, max_x-min_x)

            ball_mask = np.zeros(shape, dtype=np.uint8)
            ball_mask = cv2.circle(ball_mask, (int(x-min_x),int(y-min_y)),int(radius), (255,255,255),cv2.FILLED)

            filtered_depth = cv2.bitwise_and(section_depth_frame, section_depth_frame, mask=ball_mask)

            measure_point_count = np.count_nonzero(filtered_depth)
            if measure_point_count > 0:
                dist = np.sum(filtered_depth)/measure_point_count
                cv2.circle(debug_mask, (int(x),int(y)),int(radius), (255,255,255),cv2.FILLED)

                dist /= 1000 #mm to m

                self.locations_on_frame.append((x, y))
                locations.append(calculateCoordinate(x, y, dist, mask.shape))

        self.debug_mask = debug_mask
        return locations
