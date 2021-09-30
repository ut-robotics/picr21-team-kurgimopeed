import threading
from flask import Flask, render_template, request, Response

import time

import numpy as np

import cv2
import pyrealsense2 as rs

from threading import Thread

class RSCamera(object):
    def __init__(self):
        # configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        # TODO: configure desired shit
        # enable depth
        # 848x480 90 fps max
        # go 60
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        # enable rgb
        # 960x540 60 fps max, 1920x1080 30 possible
        self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)

        print("starting rs pipeline")
        self.pipeline.start(self.config)
        
        # should be tuple?
        self.color_frame = None
        self.depth_frame = None

        # start capture thread
        self.thread = Thread(target=self.process_frame_thread)
        self.thread.start()

    def __del__(self):
        print("stopping rs pipeline")
        self.pipeline.stop()

    def is_ready(self):
        return self.color_frame

    def process_frame_thread(self):
        while True:
            # wait for a coherent pair of frames: depth and color
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

app = Flask(__name__)

rs_cam = None

@app.route("/depth-feed")
def depth_feed():
    def feed_generator():
        while True:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.depth_frame + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not rs_cam.is_ready():
        time.sleep(0.2)
    return Response(feed_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/color-feed")
def color_feed():
    def feed_generator():
        while True:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.color_frame + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not rs_cam.is_ready():
        time.sleep(0.2)
    return Response(feed_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return render_template("index.html")

@app.before_first_request
def setup():
    global rs_cam

    # initialize camera and start frame processing
    rs_cam = RSCamera()

if __name__ == "__main__":
    app.run(debug=True, port=5000)
