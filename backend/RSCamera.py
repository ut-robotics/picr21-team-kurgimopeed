import numpy as np

import cv2
import pyrealsense2 as rs

from threading import Thread
import atexit, time

class RSCamera():
    def __init__(self):
        # configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        # TODO: find good config and load 
        #am = rs.rs400_advanced_mode(rs.context().devices().front())
        #am.load_json()


        # enable depth
        # 848x480 90 fps max
        # go 60
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        # enable rgb
        # 960x540 60 fps max, 1920x1080 30 possible
        self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)

        # flag to check in capture thread
        self.stop = False
        atexit.register(self.cleanup)

        print("starting rs pipeline")
        self.pipeline.start(self.config)
        
        # should be tuple?
        self.color_frame = None
        self.depth_frame = None

        # start capture thread
        self.thread = Thread(target=self.process_frame_thread)
        self.thread.daemon = True
        self.thread.start()

    def cleanup(self):
        # stop the pipeline sir
        print("stopping rs pipeline")
        self.stop = True
        time.sleep(0.1)
        self.pipeline.stop()

    def process_frame_thread(self):
        while not self.stop:
            # wait for a coherent pair of frames
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue # fuck

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # this is retarded, we're going to handle depth and color differently anyway
            #images = [(cv2.imencode(".jpg", x)[1]).tobytes() for x in (depth_frame, color_frame)]

            self.depth_frame = cv2.imencode(".jpg", depth_image)[1].tobytes()
            self.color_frame = cv2.imencode(".jpg", color_image)[1].tobytes()