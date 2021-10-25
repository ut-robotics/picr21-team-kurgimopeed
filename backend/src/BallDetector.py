import cv2
import numpy as np
from math import sin, cos, pi

class BallDetector():
    def __init__(self):
        self.camera_angle = -2
        self.camera_transformation = np.array([0, 0, 0.1])
        self.camera_fov = (87, 58) #H, V

        blobparams = cv2.SimpleBlobDetector_Params()

        self.detector = cv2.SimpleBlobDetector_create(blobparams)

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        degrees = (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min
        return degrees/180*pi

    def getLocations(self, mask, depth_frame):

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        keypoints = self.detector.detect(mask)

        shape = depth_frame.shape

        locations = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            radius = keypoint.size/2
            dist = np.mean(depth_frame[int(y-radius):int(y+radius),int(x-radius):int(x+radius)])
            dist /= 1000 #mm to m


            alpha = (self.linear_map(y, -shape[1]/2, shape[1]/2, -self.camera_fov[1]/2, self.camera_fov[1]/2)+self.camera_angle)/180*pi
            beeta = self.linear_map(x, -shape[0]/2, shape[0]/2, -self.camera_fov[0]/2, self.camera_fov[0]/2)/180*pi
        
            s = sin(alpha)*dist
            loc = np.array([cos(beeta)*s, cos(beeta)*s, cos(alpha)*dist])
            
            locations.append(np.add(loc, self.camera_transformation))

        return locations

