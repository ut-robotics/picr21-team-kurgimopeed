import numpy as np
import math
import cv2
import cv2.aruco as aruco

class ArucoDetector():
    def __init__(self, marker_size):
        self.marker_size = marker_size

        self.config_path = '../config/camera_calibration.npz'

        with np.load(self.config_path) as X:
            self.mtx, self.dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters =  aruco.DetectorParameters_create()

    def getMarkerLocations(self, frame, debug_frame=None):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None:
            ids = np.array([])
            
        if debug_frame is not None:
            aruco.drawDetectedMarkers(debug_frame, corners, ids)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.mtx, self.dist) #translation [x, y, z] camera to marker
                                                                                            #rotation [over x, over y, over dist(z)] camera to marker, in radians

        if len(corners) == 2:
            print("rvecs")
            for rv in rvecs:    
                print(rv)
            print("tvecs")
            for tv in tvecs:    
                print(tv)
            print("----")

        if rvecs is None:
            rvecs = np.array([])

        #if corners:
        #    frame_markers = cv2.circle(frame_markers, tuple(corners[0][0][0]), 10, (255, 0, 255), 2)

        if tvecs is None:
            tvecs = np.array([])

        markers = np.array([])
        
        marker_count = len(ids)
        if len(rvecs) == marker_count and len(tvecs) == marker_count:
            markers = np.array([{"id":ids[i][0], "tvect":tvecs[i][0], "rvect":rvecs[i][0]} for i in range(len(ids))]);
        else:
            print("Invalid count of translation or rotation vectors, should be", marker_count, "tvecs:", len(tvecs), "rvecs:", len(rvecs))

        return corners, markers #markers contain array of dictionaries {"id", "tvect", "rvect"W}

    def coordinateTransform(self, tvect, rvect):
        r_matrix, _ = cv2.Rodrigues(rvect)
        r_matrix_inv = np.linalg.inv(r_matrix)

        cam_tvect = np.matmul(r_matrix_inv, -np.transpose(tvect))
        cam_rvect = -rvect

        return np.transpose(cam_tvect), cam_rvect

    def QuaternionToDegrees(self, rvecs):
        R, _ = cv2.Rodrigues(rvecs)

        yaw = 180*math.asin(R[2][0])/math.pi
        pitch = 180*math.atan2(-R[2][1], R[2][2])/math.pi
        roll = 180*math.atan2(-R[1][0], R[0][0])/math.pi

        return pitch, roll, yaw

if __name__ == "__main__":
    detector = ArucoDetector()

    frame = cv2.imread("test.jpg")
    corners, markers = detector.getMarkerLocations(frame)
    if len(markers):
        m = markers[0]
        tcam, rcam = detector.coordinateTransform(m["tvect"], m["rvect"])
        degrees = detector.QuaternionToDegrees(rcam)
        cam_pos_str = (tcam[0], tcam[1], tcam[2], degrees[0], degrees[1], degrees[2])
        print(" xyz: %.4f %.4f %.4f\tpry: %.2f %.2f %.2f"%cam_pos_str)