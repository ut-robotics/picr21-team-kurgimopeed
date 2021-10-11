import numpy as np
import cv2

from src.RSCamera import RSCamera

from threading import Thread
import time

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
        image = np.asanyarray(frame.get_data())
        image = cv2.resize(image, (from_reso[0]//2, from_reso[1]//2))
        return cv2.imencode(".jpg", image)[1].tobytes()

    def run(self):
        self.stop = False
        super().start()
        try:
            while not self.stop:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                self.debug_frame1 = self.convert_debug_frame(depth_frame, self.depth_resolution)
                self.debug_frame2 = self.convert_debug_frame(color_frame, self.color_resolution)
                self.new_debug_frame1 = True
                self.new_debug_frame2 = True

                #print(depth_frame)
        finally:
            super().cleanup()