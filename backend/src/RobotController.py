from threading import Thread
from cv2 import RECURS_FILTER

from numpy import subtract
from src.MotorDriver import MotorDriver
from src.ImageProcess import ImageProcess
from src.LocationProcess import LocationProcess
from src.DrivingLogic import DrivingLogic
from src.RSCamera import RSCamera
from src.RefereeClient import RefereeClient

class RobotController():
    def __init__(self):
        self.rs_camera = RSCamera()
        self.rs_camera.start()

        self.motor_driver = MotorDriver()
        self.motor_driver.start() # start motor driver process
        self.driving_logic = DrivingLogic(self.motor_driver)

        self.location_process = LocationProcess()

        self.image_proccess = ImageProcess()

        self.referee_client = RefereeClient()
        self.referee_client.on_start_event(self.driving_logic.start)
        self.referee_client.on_stop_event(self.driving_logic.referee_stop)
        self.referee_client.start()

        self.stop = False
        self.thread = Thread(target=self.run)
        self.thread.daemon = True

    def run(self):
        while not self.stop:
            aligned_frames = self.rs_camera.get_aligned_frames()
            hsv_frame, depth_frame, debug_frame = self.image_proccess.get_processed_frames(aligned_frames, self.location_process)
            location = self.location_process.get_location(hsv_frame, depth_frame, debug_frame)
            #print(location["pink_goal"])
            self.driving_logic.run_logic(location)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop = True
        self.motor_driver.close()
        self.rs_camera.close()

    
