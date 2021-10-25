

from backend.src.ArucoDetector import ArucoDetector
from ArucoDetector import ArucoDetector
from BallDetector import BallDetector

class LocationProcess():
    def __init__(self):
        self.aruco_cord = (0, 0, 0)

        self.robot_location = [0, 0, 0]

        self.marker_size = 0.07 #in meters

        self.aruco = ArucoDetector(self.marker_size)
        self.ball = BallDetector()