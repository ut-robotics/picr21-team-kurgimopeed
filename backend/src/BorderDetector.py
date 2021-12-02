import cv2
import numpy as np
from math import inf, sin, cos, pi, sqrt
from src.Tools import linear_map
from src.RSCamera import camera_angle, camera_fov, camera_transformation

class BorderDetector():
    ID_BLACK = 0
    ID_WHITE = 1
    def __init__(self):
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([0, 0, 0])
        self.white_lower = np.array([0, 0, 0])
        self.white_upper = np.array([0, 0, 0])

        self.debug_mask = None

        self.location = None

    def set_threshold(self, threshold_values, id=ID_BLACK):
        if id is self.ID_BLACK:
            lower = threshold_values["black_border"][:3]
            upper = threshold_values["black_border"][3:]
            self.black_lower = np.array(lower)
            self.black_upper = np.array(upper)
        if id is self.ID_WHITE:
            lower = threshold_values["white_border"][:3]
            upper = threshold_values["white_border"][3:]
            self.white_lower = np.array(lower)
            self.white_upper = np.array(upper)

    # should ball tracking be done here?
    def get_mask(self, frame, id=ID_BLACK):
        color_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #if id is self.ID_BLACK:
        #    mask = cv2.inRange(color_hsv, self.black_lower, self.black_upper)
        #if id is self.ID_WHITE:
        mask = cv2.inRange(color_hsv, self.white_lower, self.white_upper)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        #make mask bigger
        #mask = cv2.erode(mask, np.ones((5,5),np.uint8))
        #mask = cv2.dilate(mask, np.ones((7,7),np.uint8))
        return mask

    def get_debug_mask(self):
        return self.debug_mask

    def get_coord(self, x, y, dist, shape):
        # just works (tm)
        mheight, mwidth = shape
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
        #dist_to_camera = dist
        dist_to_robot = np.linalg.norm(loc)

        return loc, dist_to_robot

    def getLocations(self, color_frame, depth_frame):
        #black_mask = self.get_mask(color_frame, self.ID_BLACK)
        white_mask = self.get_mask(color_frame, self.ID_WHITE)

        #mask = cv2.bitwise_and(black_mask, white_mask)
        mask = white_mask
        #image = cv2.rectangle(image, start_point, end_point, color, thickness)

        debug_mask = np.zeros(mask.shape, dtype=np.uint8)

        contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        
        line_end_points = []
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            _, size, _ = rect
            area_size = size[0]*size[1]
            if (area_size < 10000):
                continue

            
            box = cv2.boxPoints(rect)
            cv2.drawContours(debug_mask,[np.int0(box)],0,(255,255,255),2)

            box = sorted(box, key=lambda x: x[0])
            
            b1, b2, b3, b4 = box

            point1 = ((b1[0]+b2[0])/2, (b1[1]+b2[1])/2)
            point2 = ((b3[0]+b4[0])/2, (b3[1]+b4[1])/2)

            cv2.line(debug_mask, np.int0(point1), np.int0(point2), (255, 255, 255), 2)
            
            mheight, mwidth = mask.shape
            point1 = (max(min(point1[0], mwidth-1), 0), max(min(point1[1], mheight-1), 0))
            point2 = (max(min(point2[0], mwidth-1), 0), max(min(point2[1], mheight-1), 0))

            cord1 = self.get_coord(point1[0], point1[1], depth_frame[int(point1[1])][int(point1[0])]/1000, mask.shape)
            cord2 = self.get_coord(point2[0], point2[1], depth_frame[int(point2[1])][int(point2[0])]/1000, mask.shape)

            line_end_points.append((cord1, cord2))
            
        self.debug_mask = debug_mask
        return line_end_points

    def get_closest_dist(self, point, locations):
        smallest = float("inf")
        for line_point1, line_point2 in locations:
            cord1, _ = line_point1
            cord2, _ = line_point2
            smallest = min(smallest, np.linalg.norm(np.cross(cord2-cord1, cord1-point))/np.linalg.norm(cord2-cord1))
        return smallest

