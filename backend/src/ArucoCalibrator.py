import numpy as np
import cv2

class ArucoCalibrator():
    def __init__(self, board_size=(7, 7)):
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.board_size = board_size
        
        self.config_path = '../config/camera_calibration.npz'

        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.objp = np.zeros((self.board_size[0]*self.board_size[1],3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.board_size[0],0:self.board_size[1]].T.reshape(-1,2)

    def calibrate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (self.board_size[0],self.board_size[1]), None)

        if ret:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([self.objp], [corners], gray.shape[::-1], None, None)
            np.savez(self.config_path, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
            cv2.imwrite('../config/calibration_source.png', frame)
            print("Camera calibration saved")
            return True

        print("Calibration failed")
        return False

if __name__ == "__main__":

    frame = cv2.imread("../config/calibration_source_example.png")
    if frame is not None:
        calibration = ArucoCalibrator()
        calibration.calibrate(frame)
    else:
        print("Calibration image not found")
