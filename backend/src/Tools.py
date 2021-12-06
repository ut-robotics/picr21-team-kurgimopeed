import numpy as np
from math import cos, sin, pi
from src.RSCamera import camera_angle, camera_fov, camera_transformation

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