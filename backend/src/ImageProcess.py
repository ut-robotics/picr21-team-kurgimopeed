import numpy as np
import cv2

from src.RSCamera import RSCamera

from threading import Thread
import time, json, os

detector = cv2.SimpleBlobDetector()


class ImageProcess(RSCamera):
    def __init__(self, motor_driver):
        super().__init__()
        # start capture thread
        self.motor_driver = motor_driver

        self.stop = False
        self.thread = Thread(target=self.run)
        self.thread.daemon = True

        self.new_debug_frame1 = False
        self.debug_frame1 = np.array([])

        self.new_debug_frame2 = False
        self.debug_frame2 = np.array([])

        self.threshold_values = [0, 0, 0, 0, 0, 0]

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
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                depth_frame = np.array(depth_frame.get_data(), dtype=np.uint16)
                color_frame = np.array(color_frame.get_data(), dtype=np.uint8)

                #print(dept_frame)
                #depth_frame = np.array([[(x//256) for x in y] for y in depth_frame], dtype=np.uint8)
                #depth_frame = np.clip(depth_frame, 0, 4000)

                #depth_frame = np.vectorize(lambda x: x//0xff, otypes=[np.uint8])(depth_frame)
                #depth_frame = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.15), cv2.COLORMAP_JET)
                depth_frame = cv2.convertScaleAbs(depth_frame, alpha=0.14)
                depth_frame = cv2.bitwise_not(depth_frame)

                #depth_frame = np.asanyarray(depth_frame, dtype=np.uint8)
                #depth_frame = np.array(np.right_shift(depth_frame, 2), dtype=np.uint16 )

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

                color_mask = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
                color_mask = cv2.inRange(color_mask, lower, upper)
                #color_mask = cv2.bitwise_and(color_frame, color_frame, mask=color_mask)

                self.debug_frame1 = self.convert_debug_frame(depth_frame, self.depth_resolution)
                self.debug_frame2 = self.convert_debug_frame(color_mask, self.color_resolution)
                self.new_debug_frame1 = True
                self.new_debug_frame2 = True

                #print(depth_frame)
        finally:
            super().cleanup()