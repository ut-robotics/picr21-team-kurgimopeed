import cv2
import numpy as np
from src.Tools import calculateCoordinate, depth_distance

class BorderDetector():
    ID_BLACK = 0
    ID_WHITE = 1
    def __init__(self):
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([0, 0, 0])
        self.white_lower = np.array([0, 0, 0])
        self.white_upper = np.array([0, 0, 0])

        self.white_debug_mask = None
        self.black_debug_mask = None
        
        self.grid_size = 50

    def set_threshold(self, threshold_values, id=ID_WHITE):
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
    def get_mask(self, hsv_frame, id=ID_WHITE):
        if id is self.ID_BLACK:
            mask = cv2.inRange(hsv_frame, self.black_lower, self.black_upper)
        elif id is self.ID_WHITE:
            mask = cv2.inRange(hsv_frame, self.white_lower, self.white_upper)

        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel=np.ones((5,5),np.uint8))

        #make mask bigger
        #mask = cv2.erode(mask, np.ones((5,5),np.uint8))
        #mask = cv2.dilate(mask, np.ones((7,7),np.uint8))
        return mask

    def get_debug_mask(self, id):
        if id is self.ID_WHITE:
            return self.white_debug_mask
        if id is self.ID_BLACK:
            return self.black_debug_mask

    def generate_area_of_interest(self, frame_locations, shape):
        mask = np.zeros(shape, dtype=np.uint8)
        area_size = 300
        for x, y in frame_locations:
            start = (x-area_size, y-area_size)
            end =  (x+area_size, y+area_size)
            cv2.rectangle(mask, np.int0(start), np.int0(end), (255, 255, 255), -1)
        return mask

    def getLocations(self, hsv_frame, depth_frame, id=ID_WHITE, area_of_interest=None):
        if id is self.ID_BLACK:
            mask = self.get_mask(hsv_frame, self.ID_BLACK)
        elif id is self.ID_WHITE:
            mask = self.get_mask(hsv_frame, self.ID_WHITE)

        if area_of_interest is not None:
            mask = cv2.bitwise_and(mask, mask, mask=area_of_interest)

        mheight, mwidth = mask.shape

        for x in range(0, mwidth, self.grid_size):
            mask = cv2.line(mask, (int(x), 0), (int(x), mheight), (0, 0, 0), 1)
        for y in range(0, mheight, self.grid_size):
            mask = cv2.line(mask, (0, int(y)), (mwidth, int(y)), (0, 0, 0), 1)

        debug_mask = np.zeros(mask.shape, dtype=np.uint8)

        contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        
        line_locations = []
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            _, size, _ = rect
            area_size = size[0]*size[1]
            if (area_size < 50):
                continue

            box = cv2.boxPoints(rect)

            dist = depth_distance(box, depth_frame)
            if not np.isnan(dist):
                cv2.drawContours(debug_mask,[np.int0(box)],0,(255,255,255),2)

                x_center = np.mean([i[0] for i in box])
                y_center = np.mean([i[1] for i in box])

                dist /= 1000 #mm to m

                line_locations.append(calculateCoordinate(x_center, y_center, dist, mask.shape))
            
        if id is self.ID_WHITE:
            self.white_debug_mask = debug_mask
        elif id is self.ID_BLACK:
            self.black_debug_mask = debug_mask
            
        return line_locations

    def get_closest_dist(self, point, locations):
        smallest = float("inf")
        for line_point, dist in locations:
            smallest = min(smallest, np.linalg.norm(point-line_point))
        return smallest

