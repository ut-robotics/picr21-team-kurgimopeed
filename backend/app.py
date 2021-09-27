from flask import Flask, render_template, request, Response

import time

import numpy as np

import cv2
import pyrealsense2 as rs

from threading import Thread

class RSCamera(object):
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == "L500":
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)
        self.thread = Thread(target=self.process_frame_thread)
        self.thread.start()

        self.rgb_frame = None

    def __del__(self):
        print("stopping rs pipeline")
        self.pipeline.stop()

    def process_frame_thread(self):
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            ret, jpeg = cv2.imencode(".jpg", color_image)
            self.rgb_frame = jpeg.tobytes()

app = Flask(__name__)

rs_cam = None

# todo this serves current frame, too tired to debug
def feed_generator():
    while True:
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.rgb_frame + b"\r\n\r\n")

@app.route("/rs")
def video_feed():
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