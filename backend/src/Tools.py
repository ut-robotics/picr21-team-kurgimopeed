import numpy as np
from math import cos, sin, pi
from src.RSCamera import camera_angle, camera_fov, camera_transformation
import cv2

def linear_map(x, x_min, x_max, y_min, y_max):
    return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min

def calculateCoordinate(x, y, dist, frame_shape):
        # just works (tm)
        mheight, mwidth = frame_shape #frame shape for FOV calculation
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

def depth_distance(boxPoints, depth_frame):
    x_points = [int(x) for x, y in boxPoints]
    y_points = [int(y) for x, y in boxPoints]

    min_x, max_x = min(x_points), max(x_points)
    min_y, max_y = min(y_points), max(y_points)

    if min_x < 0:
        min_x = 0
    if max_x > depth_frame.shape[1]-1:
        max_x = depth_frame.shape[1]-1
    if min_y < 0:
        min_y = 0
    if max_y > depth_frame.shape[0]-1:
        max_y = depth_frame.shape[0]-1
 
    depth_frame = depth_frame[min_y:max_y, min_x:max_x]

    shape = (max_y-min_y, max_x-min_x)

    new_boxPoints = [(x-min_x, y-min_y) for x, y in boxPoints]

    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, pts=[np.int0(new_boxPoints)], color=(255,255,255))

    filtered_depth = cv2.bitwise_and(depth_frame, depth_frame, mask=mask)
    measure_point_count = np.count_nonzero(filtered_depth)

    return np.sum(filtered_depth)/measure_point_count