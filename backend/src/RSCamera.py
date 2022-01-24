import numpy as np
import pyrealsense2 as rs

camera_angle = -20
camera_transformation = np.array([0, 0, 0.190])
camera_fov = (87, 58) #H, V

class RSCamera():
    def __init__(self):
        # configure depth and color streams

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        #self.device.hardware_reset()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        # TODO: find good config and load 
        #am = rs.rs400_advanced_mode(rs.context().devices().front())
        #am.load_json()

        self.depth_resolution = (848, 480)
        self.color_resolution = (960, 540)
        #self.color_resolution = (1920, 1080)

        # enable depth
        # 848x480 90 fps max
        # go 60
        x, y = self.depth_resolution
        self.config.enable_stream(rs.stream.depth, x, y, rs.format.z16, 60)
        # enable rgb
        # 960x540 60 fps max, 1920x1080 30 possible
        x, y = self.color_resolution
        self.config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, 60)
        #self.config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, 30)

        #atexit.register(self.cleanup)

        # should be tuple?
        self.color_frame = None
        self.depth_frame = None

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def start(self):
        self.pipeline.start(self.config)
        print("started rs pipeline")

    def close(self):
        # stop the pipeline sir
        print("stopping rs pipeline")
        self.pipeline.stop()

    def get_aligned_frames(self):
        aligned_frames = self.align.process(self.pipeline.wait_for_frames())

        return (
            np.asanyarray(aligned_frames.get_depth_frame().get_data(), dtype=np.uint16),
            np.asanyarray(aligned_frames.get_color_frame().get_data(), dtype=np.uint8)
        )