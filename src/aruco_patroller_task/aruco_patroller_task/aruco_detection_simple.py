import cv2
import cv2.aruco as aruco

image = cv2.imread("bitmap.jpg")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

if ids is not None:
    print(f"Detected markers: {len(ids)}")
    print("IDs:", ids.flatten().tolist())
    aruco.drawDetectedMarkers(image, corners, ids)

cv2.imshow("ArUco detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()