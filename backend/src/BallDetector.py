import cv2
import numpy as np
from math import sin, cos, pi, sqrt
from src.Tools import linear_map
from src.RSCamera import camera_angle, camera_fov, camera_transformation

class BallDetector():
    def __init__(self):
        self.threshold_lower = np.array([0, 0, 0])
        self.threshold_upper = np.array([0, 0, 0])

        blobparams = cv2.SimpleBlobDetector_Params()

        blobparams.filterByColor = True
        blobparams.blobColor = 255

        #blobparams.filterByArea = True
        blobparams.minArea = 100
        blobparams.maxArea = 100000

        #blobparams.minDistBetweenBlobs = 100

        #blobparams.filterByCircularity = True
        blobparams.minCircularity = 0.8

        blobparams.filterByConvexity = False
        blobparams.minConvexity = 0.5

        blobparams.filterByInertia = False
        blobparams.minInertiaRatio = 0.01

        self.detector = cv2.SimpleBlobDetector_create(blobparams)

        self.keypoints = []
        self.debug_mask = None

    def set_threshold(self, lower, upper):
        self.threshold_lower = np.array([lower])
        self.threshold_upper = np.array([upper])

    def get_mask(self, frame):
        color_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(color_hsv, self.threshold_lower, self.threshold_upper)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask


    # should ball tracking be done here?
    def getLocations(self, color_frame, depth_frame):
        mask = self.get_mask(color_frame)
        mheight, mwidth = mask.shape

        self.debug_mask = np.zeros(mask.shape, dtype=np.uint8)

        keypoints = self.detector.detect(mask)
        self.keypoints = keypoints

        locations = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            radius = keypoint.size/2

            ball_mask = np.zeros(mask.shape, dtype=np.uint8)
            ball_mask = cv2.circle(ball_mask, (int(x),int(y)),int(radius), (255,255,255),cv2.FILLED)

            resized_depth = cv2.resize(depth_frame, (mwidth, mheight))
            filtered_depth = cv2.bitwise_and(resized_depth, resized_depth, mask=ball_mask)

            measure_point_count = np.count_nonzero(filtered_depth)
            if measure_point_count > 0:
                dist = np.sum(filtered_depth)/measure_point_count
                self.debug_mask = cv2.bitwise_or(self.debug_mask, ball_mask)

                dist /= 1000 #mm to m

                # just works (tm)
                alpha = (linear_map(y, mheight, 0, -camera_fov[1]/2, camera_fov[1]/2) + camera_angle - 90.0)/180*pi
                beeta = linear_map(x, mwidth, 0, -camera_fov[0]/2, camera_fov[0]/2)/180*pi

                s = sin(alpha)*dist

                cord_x = sin(beeta) * s
                cord_y = -cos(beeta) * s
                cord_z = cos(alpha) * dist

                loc = np.add(np.array([cord_x, cord_y, cord_z]), camera_transformation)
                #print(loc)

                # vector magnitude
                #depth camera already measures dist to camera
                dist_to_robot = np.linalg.norm(loc)
                # a dictionary with ids would be better
                locations.append((loc, dist_to_robot))

        return locations
