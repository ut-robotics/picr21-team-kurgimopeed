from cv2 import transform
from src.ArucoDetector import ArucoDetector
from src.BallDetector import BallDetector
from src.GoalDetector import GoalDetector
from src.BorderDetector import BorderDetector

from math import pi, cos, sin
import numpy as np

class LocationProcess():
    def __init__(self):
        #x positive is in blue pillar direction
        #y positive is moving far away, if blue pillar is in right
        #z positive is moving to the sky
        self.aruco_locations = {
            11: np.array([-2.27, -0.15, 0.105], dtype=np.float64), #pink basket left
            12: np.array([-2.27, 0.16, 0.105], dtype=np.float64), #pink basket right
            21: np.array([2.27, 0.145, 0.105], dtype=np.float64), #blue basket left 
            22: np.array([2.27, -0.155, 0.105], dtype=np.float64)  #blue basket right
        }

        self.aruco_rotations = {
            11: np.array([0, -90, 90], dtype=np.float64), #pink basket left
            12: np.array([0, -90, 90], dtype=np.float64), #pink basket right
            21: np.array([0, 90, -90], dtype=np.float64), #blue basket left 
            22: np.array([0, 90, -90], dtype=np.float64)  #blue basket right
        }

        self.robot_location = [0, 0, 0]
        self.robot_rotation = [0, 0, 0]

        self.marker_size = 0.158 #in meters
        #self.marker_size = 0.04
        self.aruco = ArucoDetector(self.marker_size)
        self.ball = BallDetector()
        self.goal = GoalDetector()
        self.border = BorderDetector()

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

    def get_location(self, hsv_frame, depth, debug_frame=None):
        ''' _, markers = self.aruco.getMarkerLocations(hsv_frame, debug_frame=debug_frame)

        if len(markers)>0:
            correct_marker_count = 0
            transform = np.array([0, 0, 0], dtype=np.float64)
            rotation = np.array([0, 0, 0], dtype=np.float64)

            for m in markers:
                if m["id"] in self.aruco_locations: #if such aruco id exists
                    tcam, rcam = self.aruco.coordinateTransform(m["tvect"], m["rvect"])
                    degrees = self.aruco.QuaternionToDegrees(rcam)
                    cam_pos_str = (tcam[0], tcam[1], tcam[2], degrees[0], degrees[1], degrees[2])
                    #print(" xyz:\t%.4f\t%.4f\t%.4f\tpry:\t%.2f\t%.2f\t%.2f"%cam_pos_str)

                    aruco_transform = self.rotation_transform(tcam, self.aruco_rotations[m["id"]])
                    transform += aruco_transform+self.aruco_locations[m["id"]]
                    rotation += degrees

                    correct_marker_count += 1
                else:
                    print("Found unknown aruco id %d"%m["id"])

            self.robot_location = np.divide(transform, correct_marker_count)
            self.robot_rotation = np.divide(rotation, correct_marker_count)

        else:
            pass
            #location handler if no markers found
        '''

        #balls = []
        # TODO: i am unsure about this fn, comment back in after proper localization?
        # -josh
        #ball_locations = [(self.rotation_transform(ball[0], self.robot_rotation), ball[1]) for ball in balls]
        #ball_locations = balls

        ball_locations = self.ball.getLocations(hsv_frame, depth)
        pink_goal = self.goal.getLocations(hsv_frame, depth, id=self.goal.ID_PINK)
        blue_goal = self.goal.getLocations(hsv_frame, depth, id=self.goal.ID_BLUE)

        area_of_interest = self.border.generate_area_of_interest(self.ball.locations_on_frame, hsv_frame.shape[:2])
        white_border_points = self.border.getLocations(hsv_frame, depth, self.border.ID_WHITE, area_of_interest)
        black_border_points = self.border.getLocations(hsv_frame, depth, self.border.ID_BLACK, area_of_interest)
        border_dist = []

        for i in ball_locations:
            loc, dist = i
            w = self.border.get_closest_dist(loc, white_border_points)
            b = self.border.get_closest_dist(loc, black_border_points)
            s = b-w
            if np.isnan(s):
                s = 0
            border_dist.append(s)
        
        #print(ball_locations)
        #print(border_dist)
        #filter(lambda n, x: border_dist[n] >=0, range(ball_locations))
        
        return {"robot_loc":self.robot_location, 
                "robot_rot":self.robot_rotation,
                "pink_goal":pink_goal,
                "blue_goal":blue_goal,
                "balls":ball_locations}
