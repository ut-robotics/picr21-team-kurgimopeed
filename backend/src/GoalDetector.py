import cv2
import numpy as np
from math import sin, cos, pi, sqrt
from src.Tools import linear_map
from src.RSCamera import camera_angle, camera_fov, camera_transformation

from threading import Thread

class GoalDetector():
    ID_PINK = 0
    ID_BLUE = 1
    def __init__(self):
        self.pink_lower = np.array([0, 0, 0])
        self.pink_upper = np.array([0, 0, 0])
        self.blue_lower = np.array([0, 0, 0])
        self.blue_upper = np.array([0, 0, 0])

        self.pink_debug_mask = None
        self.blue_debug_mask = None

        self.pink_location = None
        self.blue_location = None

    def set_threshold(self, threshold_values, id=ID_PINK):
        if id is self.ID_PINK:
            lower = threshold_values["pink_goal"][:3]
            upper = threshold_values["pink_goal"][3:]
            self.pink_lower = np.array(lower)
            self.pink_upper = np.array(upper)
        if id is self.ID_BLUE:
            lower = threshold_values["blue_goal"][:3]
            upper = threshold_values["blue_goal"][3:]
            self.blue_lower = np.array(lower)
            self.blue_upper = np.array(upper)

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

    def get_debug_mask(self, id=ID_PINK):
        if (id == self.ID_PINK):
            return self.pink_debug_mask
        if (id == self.ID_BLUE):
            return self.blue_debug_mask

    def start_process(self, color, depth, id):
        process = Thread(target=self.getLocations, args=(color, depth, id))
        process.start()
        return process

    def join(self, process: Thread, id):
        #locations, self.debug_mask = self.process_queue.get()
        process.join()
        if id is self.ID_PINK:
            return self.pink_location
        elif id is self.ID_BLUE:
            return self.blue_location
        return None

    def getLocations(self, color_frame, depth_frame, id=ID_PINK):
        mask = self.get_mask(color_frame, id)
        mheight, mwidth = mask.shape

        if (id == self.ID_PINK):
            self.pink_debug_mask = np.zeros(mask.shape, dtype=np.uint8)
        if (id == self.ID_BLUE):
            self.blue_debug_mask = np.zeros(mask.shape, dtype=np.uint8)

        contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        
        goal = None
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            _, size, _ = rect
            area_size = size[0]*size[1]
            g_area_size = 0
            if goal is not None:
                _, g_size, _ = goal
                g_area_size = g_size[0]*g_size[1]
                
            if g_area_size < area_size:
                if (area_size > 1000):
                    goal = rect


        if goal is not None:
            box = cv2.boxPoints(goal)
            x_center = np.mean([i[0] for i in box])
            y_center = np.mean([i[1] for i in box])

            goal_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.fillPoly(goal_mask, pts=[np.int0(box)], color=(255,255,255))

            resized_depth = cv2.resize(depth_frame, (mwidth, mheight))

            filtered_depth = cv2.bitwise_and(resized_depth, resized_depth, goal_mask)

            measure_point_count = np.count_nonzero(filtered_depth)
            if measure_point_count > 0:
                dist = np.sum(filtered_depth)/measure_point_count
                if (id == self.ID_PINK):
                    self.pink_debug_mask = goal_mask
                if (id == self.ID_BLUE):
                    self.blue_debug_mask = goal_mask

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
                #dist_to_camera = dist
                #dist_to_camera = np.linalg.norm(loc)
                # a dictionary with ids would be better
                if id is self.ID_PINK:
                    self.pink_location = loc
                if id is self.ID_BLUE:
                    self.blue_location = loc
                #return loc

        #return None
