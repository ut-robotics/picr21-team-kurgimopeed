import numpy as np
import cv2
from src.LocationProcess import LocationProcess

from src.RSCamera import RSCamera
from src.ArucoCalibrator import ArucoCalibrator
from src.LocationProcess import LocationProcess

from threading import Thread
import time, json, os

import copy as cp

detector = cv2.SimpleBlobDetector()
calibrator = ArucoCalibrator()


class ImageProcess(RSCamera):
    def __init__(self, driving_logic):
        super().__init__()
        # start capture thread
        self.driving_logic = driving_logic 

        self.stop = False
        self.thread = Thread(target=self.run)
        self.thread.daemon = True

        self.color_frame = np.array([])
        self.depth_frame = np.array([])

        self.new_debug_frame1 = False
        self.debug_frame1 = np.array([])

        self.new_debug_frame2 = False
        self.debug_frame2 = np.array([])

        self.show_mask = True

        self.trackbar_path = "../config/threshold_config.json"

        self.threshold_values = [0, 0, 0, 255, 255, 255]
        if os.path.exists(self.trackbar_path):
            with open("../config/threshold_config.json") as f:
                self.threshold_values = list(json.load(f).values())

        self.locationProcess = LocationProcess()

    def start(self):
        self.thread.start()
        print("Image processing started")

    def stop(self):
        self.stop = True
        self.thread.join()
        print("Image processing stopped")

    def has_new_frame1(self):
        ret = self.new_debug_frame1
        self.new_debug_frame1 = False
        return ret

    def get_frame1(self):
        return self.debug_frame1

    def has_new_frame2(self):
        ret = self.new_debug_frame2
        self.new_debug_frame2 = False
        return ret

    def get_frame2(self):
        return self.debug_frame2

    def convert_debug_frame(self, frame, from_reso=(640, 480)):
        image = cv2.resize(frame, (from_reso[0]//2, from_reso[1]//2))
        return cv2.imencode(".jpg", image)[1].tobytes()

    def run(self):
        self.stop = False
        super().start()

        try:
            while not self.stop:
                frames = self.pipeline.wait_for_frames()

                #https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
                aligned_frames = self.align.process(frames)

                # Get aligned frames
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                #depth_frame = frames.get_depth_frame()
                #color_frame = frames.get_color_frame()

                self.depth_frame = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)
                self.color_frame = np.asanyarray(color_frame.get_data(), dtype=np.uint8)

                debug1 = cp.deepcopy(self.depth_frame)
                debug2 = cp.deepcopy(self.color_frame)

                #print(dept_frame)
                #depth_frame = np.array([[(x//256) for x in y] for y in depth_frame], dtype=np.uint8)
                #depth_frame = np.clip(depth_frame, 0, 4000)

                #depth_frame = np.vectorize(lambda x: x//0xff, otypes=[np.uint8])(depth_frame)
                #depth_frame = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.15), cv2.COLORMAP_JET)
                #depth_frame = cv2.convertScaleAbs(self.depth_frame, alpha=0.14)
                #depth_frame = cv2.bitwise_not(depth_frame)


                # yes
                """
                curr = os.stat("../config/threshold_config.json").st_mtime
                if changed != curr:
                    changed = curr
                    with open("../config/threshold_config.json", "r") as f:
                        tc = list(json.load(f).values())
                """

                lower = np.array(self.threshold_values[:3])
                upper = np.array(self.threshold_values[3:])

                color_hsv = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2HSV)
                color_mask = cv2.inRange(color_hsv, lower, upper)

                kernel = np.ones((5,5),np.uint8)
                color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)

                #scale it to see better visualization
                debug1 = cv2.convertScaleAbs(debug1, alpha=0.14)
                debug1 = cv2.cvtColor(cv2.bitwise_not(debug1), cv2.COLOR_GRAY2BGR)

                if self.show_mask:
                    debug2 = cv2.bitwise_and(debug2, debug2, mask=color_mask)

                #add debug data
                debug2 = cv2.drawKeypoints(debug2, self.locationProcess.ball.keypoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                xl, xr, yu, yd = self.locationProcess.ball.depth_area
                debug1 = cv2.rectangle(debug1, (xl, yu), (xr, yd), (0, 0, 255), 2)

                location = self.locationProcess.get(self.color_frame, self.depth_frame, color_mask, debug_frame=debug2)
                #print(len(location["balls"])) # temp remove, put back if necessary - josh
                
                self.debug_frame1 = self.convert_debug_frame(debug1, self.depth_resolution)
                self.debug_frame2 = self.convert_debug_frame(debug2, self.color_resolution)

                self.new_debug_frame1 = True
                self.new_debug_frame2 = True

                self.driving_logic.run_logic(location) # params

                #print(depth_frame)
        finally:
            super().cleanup()

    def calibrate(self):
        return calibrator.calibrate(self.color_frame)
