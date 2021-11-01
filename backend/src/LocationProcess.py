from src.ArucoDetector import ArucoDetector
from src.BallDetector import BallDetector

from math import pi, cos, sin
import numpy as np

class LocationProcess():
    def __init__(self):
        self.aruco_cord = (0, 0, 0)

        self.robot_location = [0, 0, 0]
        self.robot_rotation = [0, 0, 0]

        self.marker_size = 0.07 #in meters

        self.aruco = ArucoDetector(self.marker_size)
        self.ball = BallDetector()

    def rotation_transform(self, cord, rot):
        x_rot, y_rot, z_rot = [i/180*pi for i in rot] #converion to radians

        x_axis_rot_matrix = np.array((
                                    (0, 0, 1),
                                    (0, cos(x_rot), -sin(x_rot)),
                                    (0, sin(x_rot), cos(x_rot))
                                   ))
        y_axis_rot_matrix = np.array((
                                    (cos(y_rot), 0, sin(y_rot)),
                                    (0, 0, 1),
                                    (-sin(y_rot), 0, cos(y_rot))
                                   ))                                 
        z_axis_rot_matrix = np.array((
                                    (cos(z_rot), -sin(z_rot), 0),
                                    (sin(z_rot), cos(z_rot), 0),
                                    (0, 0, 1)
                                   ))

        cord = np.matmul(x_axis_rot_matrix, cord)
        cord = np.matmul(y_axis_rot_matrix, cord)
        cord = np.matmul(z_axis_rot_matrix, cord)

        return cord

    def get(self, mask, depth):
        balls = self.ball.getLocations(mask, depth)
        ball_locations = [self.rotation_transform(ball, self.robot_rotation) for ball in balls]
        return {"robot_loc":self.robot_location, 
                "robot_rot":self.robot_rotation,
                "balls":ball_locations}
