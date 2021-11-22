import cv2
import numpy as np
from math import sin, cos, pi, sqrt
from src.Tools import linear_map
from src.RSCamera import camera_angle, camera_fov, camera_transformation

class GoalDetector():
    ID_PINK = 0
    ID_BLUE = 1
    def __init__(self):
        self.pink_lower = np.array([0, 0, 0])
        self.pink_upper = np.array([0, 0, 0])
        self.blue_lower = np.array([0, 0, 0])
        self.blue_upper = np.array([0, 0, 0])

        self.goal = None
        self.depth_area = [0, 0, 0, 0]

    def set_threshold(self, lower, upper, id=ID_PINK):
        if id is self.ID_PINK:
            self.pink_lower = np.array(lower)
            self.pink_upper = np.array(upper)
        if id is self.ID_BLUE:
            self.blue_lower = np.array(lower)
            self.pink_upper = np.array(upper)

    # should ball tracking be done here?
    def get_mask(self, frame, id=ID_PINK):
        color_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if id is self.ID_PINK:
            mask = cv2.inRange(color_hsv, self.pink_lower, self.pink_upper)
        if id is self.ID_BLUE:
            mask = cv2.inRange(color_hsv, self.blue_lower, self.blue_upper)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def getLocations(self, color_frame, depth_frame, id=ID_PINK):
        mask = self.get_mask(color_frame, id)
        mheight, mwidth = mask.shape

        contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        
        self.goal = None
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            _, size, _ = rect
            area_size = size[0]*size[1]
            g_area_size = 0
            if self.goal is not None:
                _, g_size, _ = self.goal
                g_area_size = g_size[0]*g_size[1]
                
            if g_area_size < area_size:
                if (area_size > 1000):
                    self.goal = rect

        if self.goal is not None:
            box = cv2.boxPoints(self.goal)
            x_center = np.mean([i[0] for i in box])
            y_center = np.mean([i[1] for i in box])

            goal_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.fillPoly(goal_mask, pts=[np.int0(box)], color=(255,255,255))

            resized_depth = cv2.resize(depth_frame, (mwidth, mheight))
            filtered_depth = cv2.bitwise_and(resized_depth, resized_depth, goal_mask)

            measure_point_count = np.count_nonzero(filtered_depth)
            if measure_point_count > 0:
                dist = np.sum(filtered_depth)/measure_point_count
                self.debug_mask = cv2.bitwise_or(self.debug_mask, goal_mask)

                dist /= 1000 #mm to m

                # just works (tm)
                alpha = (linear_map(y_center, mheight, 0, -camera_fov[1]/2, camera_fov[1]/2) + camera_angle - 90.0)/180*pi
                beeta = linear_map(x_center, mwidth, 0, -camera_fov[0]/2, camera_fov[0]/2)/180*pi

                s = sin(alpha)*dist

                cord_x = sin(beeta) * s
                cord_y = -cos(beeta) * s
                cord_z = cos(alpha) * dist

                loc = np.add(np.array([cord_x, cord_y, cord_z]), camera_transformation)
                #print(loc)

                # vector magnitude
                #depth camera already measures dist to camera
                dist_to_camera = dist
                #dist_to_camera = np.linalg.norm(loc)
                # a dictionary with ids would be better
                return loc, dist_to_camera

        return None
