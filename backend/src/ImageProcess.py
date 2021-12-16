import numpy as np
import cv2

from src.ArucoCalibrator import ArucoCalibrator

import time, json, os

import copy as cp

detector = cv2.SimpleBlobDetector()
calibrator = ArucoCalibrator()

class ImageProcess():
    def __init__(self):
        self.color_frame = np.array([])
        self.depth_frame = np.array([])

        self.new_debug_frame1 = False
        #self.debug_frame1 = np.array([])

        self.new_debug_frame2 = False
        #self.debug_frame2 = np.array([])

        self.show_mask = True

        self.trackbar_path = "../config/threshold_config.json"

        self.threshold_values = {
            "off": [0, 0, 0, 0, 0, 0],
            "balls": [0, 0, 0, 255, 255, 255],
            "pink_goal": [0, 0, 0, 255, 255, 255],
            "blue_goal": [0, 0, 0, 255, 255, 255],
            "black": [0, 0, 0, 255, 255, 255],
        }
        if os.path.exists(self.trackbar_path):
            with open("../config/threshold_config.json") as f:
                self.threshold_values = json.load(f)
        self.active_threshold_config = list(self.threshold_values.keys())[0]

        self.fps_start_time = 0

    def has_new_frame1(self):
        ret = self.new_debug_frame1
        self.new_debug_frame1 = False
        return ret

    #def get_frame1(self):
    #    return self.debug_frame1

    def has_new_frame2(self):
        ret = self.new_debug_frame2
        self.new_debug_frame2 = False
        return ret

    #def get_frame2(self):
    #    return self.debug_frame2

    #def convert_debug_frame(self, frame, from_reso=(640, 480)):
    #    return None
        #return cv2.imencode(".jpg", frame)[1].tobytes()
        #image = cv2.resize(frame, (from_reso[0]//2, from_reso[1]//2))
        #return cv2.imencode(".jpg", image)[1].tobytes()

    def draw_mask_on_frame(self, frame, mask, color):
        solid_color = np.zeros((mask.shape[0], mask.shape[1], 3), np.uint8)
        solid_color[:] = color
        ball_debug = cv2.bitwise_and(frame, frame, mask=cv2.bitwise_not(mask))
        ball_debug_color = cv2.bitwise_and(solid_color, solid_color, mask=mask)
        return np.add(ball_debug, ball_debug_color)

    def get_processed_frames(self, aligned_frames, location_process):
        fps_current_time = time.time()
        dtime = fps_current_time-self.fps_start_time
        self.fps_start_time = fps_current_time
        fps = round(1/dtime)

        print(f"{fps=}", end='\r')

        self.depth_frame, self.color_frame = aligned_frames

        debug1 = cp.deepcopy(self.depth_frame)
        debug2 = cp.deepcopy(self.color_frame)

        ball = location_process.ball
        ball.set_threshold(self.threshold_values)

        goal = location_process.goal
        goal.set_threshold(self.threshold_values, id=goal.ID_BLUE)
        goal.set_threshold(self.threshold_values, id=goal.ID_PINK)

        border = location_process.border
        border.set_threshold(self.threshold_values, id=border.ID_BLACK)
        border.set_threshold(self.threshold_values, id=border.ID_WHITE)

        #scale it to see better visualization
        debug1 = cv2.convertScaleAbs(debug1, alpha=0.14)
        debug1 = cv2.cvtColor(cv2.bitwise_not(debug1), cv2.COLOR_GRAY2BGR)
        
        hsv_frame = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2HSV)
        #add debug data
        if self.show_mask:
            if self.active_threshold_config == "pink_goal":
                goal_mask = goal.get_mask(hsv_frame, id=goal.ID_PINK)
                debug2 = cv2.bitwise_and(debug2, debug2, mask=goal_mask)
            if self.active_threshold_config == "blue_goal":
                goal_mask = goal.get_mask(hsv_frame, id=goal.ID_BLUE)
                debug2 = cv2.bitwise_and(debug2, debug2, mask=goal_mask)
            if self.active_threshold_config == "balls":
                ball_mask = ball.get_mask(hsv_frame)
                debug2 = cv2.bitwise_and(debug2, debug2, mask=ball_mask)
            if self.active_threshold_config == "black_border":
                border_mask = border.get_mask(hsv_frame, id=border.ID_BLACK)
                debug2 = cv2.bitwise_and(debug2, debug2, mask=border_mask)
            if self.active_threshold_config == "white_border":
                border_mask = border.get_mask(hsv_frame, id=border.ID_WHITE)
                debug2 = cv2.bitwise_and(debug2, debug2, mask=border_mask)
        else:
            if self.active_threshold_config == "pink_goal":
                pink_goal_mask = goal.get_debug_mask(id=goal.ID_PINK)
                if pink_goal_mask is not None:
                    debug2 = self.draw_mask_on_frame(debug2, pink_goal_mask, (255, 0, 255))

            if self.active_threshold_config == "blue_goal":
                blue_goal_mask = goal.get_debug_mask(id=goal.ID_BLUE)
                if blue_goal_mask is not None:
                    debug2 = self.draw_mask_on_frame(debug2, blue_goal_mask, (255, 0, 0))

            if self.active_threshold_config == "balls":
                ball_mask = ball.get_debug_mask()
                if ball_mask is not None:
                    debug2 = self.draw_mask_on_frame(debug2, ball_mask, (0, 255, 0))

            if self.active_threshold_config == "white_border" or self.active_threshold_config == "black_border":
                border_mask = border.get_debug_mask(border.ID_WHITE)
                if border_mask is not None:
                    debug2 = self.draw_mask_on_frame(debug2, border_mask, (255, 255, 255))

                border_mask = border.get_debug_mask(border.ID_BLACK)
                if border_mask is not None:
                    debug2 = self.draw_mask_on_frame(debug2, border_mask, (0, 0, 0))
                
        cv2.putText(debug2, "%sFPS"%(fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        self.debug_frame1 = debug1
        self.debug_frame2 = debug2

        self.new_debug_frame1 = True
        self.new_debug_frame2 = True

        return hsv_frame, self.depth_frame, debug2

    def calibrate(self):
        return calibrator.calibrate(self.color_frame)
