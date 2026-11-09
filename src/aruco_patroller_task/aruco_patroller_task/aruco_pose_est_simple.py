import cv2
import cv2.aruco as aruco
import numpy as np

image = cv2.imread("bitmap.jpg")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

camera_matrix = np.array([[600.0,   0.0, 320.0],
                          [  0.0, 600.0, 240.0],
                          [  0.0,   0.0,   1.0]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)
marker_length = 0.05

if ids is not None:
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, marker_length, camera_matrix, dist_coeffs
    )
    for rvec, tvec, id in zip(rvecs, tvecs, ids):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3] = tvec[0]
        print(f"Marker ID: {id[0]}")
        print("T_marker_camera:\n", T)
        