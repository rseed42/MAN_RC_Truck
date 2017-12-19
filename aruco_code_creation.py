import numpy as np
import cv2
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
img = aruco.drawMarker(aruco_dict, 42, 1000)
cv2.imwrite("aruco42.jpg", img)
