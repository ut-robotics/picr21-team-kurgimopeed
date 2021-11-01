import cv2
import numpy as np
from math import sin, cos, pi, sqrt
from src.Tools import linear_map

class BallDetector():
    def __init__(self):
        self.camera_angle = 0
        self.camera_transformation = np.array([0, 0, 0.0])
        self.camera_fov = (87, 58) #H, V

        blobparams = cv2.SimpleBlobDetector_Params()

        blobparams.filterByColor = True
        blobparams.blobColor = 255

        #blobparams.filterByArea = True
        blobparams.minArea = 10
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
        self.depth_area = [0, 0, 0 ,0]

    def getLocations(self, mask, depth_frame):

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        keypoints = self.detector.detect(mask)
        self.keypoints = keypoints

        #depth and color cameras are with different zoom and shifted, need pixel wrapper
        dheight, dwidth = depth_frame.shape
        mheight, mwidth = mask.shape

        locations = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            radius = keypoint.size/2/sqrt(2) #circles inner square half edge length

            rect_x_left = int(linear_map(x-radius, 0, mwidth, 0, dwidth))
            rect_x_right = int(linear_map(x+radius, 0, mwidth, 0, dwidth))
            rect_y_up = int(linear_map(y-radius, 0, mheight, 0, dheight))
            rect_y_down = int(linear_map(y+radius, 0, mheight, 0, dheight))
            self.depth_area = [rect_x_left, rect_x_right, rect_y_up, rect_y_down]

            depth_ball_area = depth_frame[rect_y_up:rect_y_down, rect_x_left:rect_x_right]
            if len(depth_ball_area)>0:
                dist = np.mean(depth_ball_area)
                dist /= 1000 #mm to m

                alpha = (linear_map(y, 0, mheight, -self.camera_fov[1]/2, self.camera_fov[1]/2)+self.camera_angle)/180*pi
                beeta = linear_map(x, 0, mwidth, -self.camera_fov[0]/2, self.camera_fov[0]/2)/180*pi
            
                s = sin(alpha)*dist

                cord_z = -cos(beeta)*s
                cord_y = sin(beeta)*s
                cord_x = cos(alpha)*dist

                loc = np.array([cord_x, cord_y, cord_z])
                locations.append(np.add(loc, self.camera_transformation))

        return locations

