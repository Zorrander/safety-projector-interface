#!/usr/bin/env python3
#this code project an aruco grid, detect them on the camera and send them tot he ROS service

import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl




if __name__ == "__main__":
	aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
	
	testim = np.zeros((1080, 1920, 1), dtype="uint8")+255 

	cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
	#change second parameter to fit your screen resolution
	cv2.moveWindow("window", 2560, 0)
	cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

	size = 100
	aruc_id = 0

	for y in range(300,800,200):
		for x in range(300,1500,225):
			print(x, y)

	"""		
	for y in range(300,800,200):
		for x in range(300,1500,225):
			tag = np.zeros((size, size, 1), dtype="uint8")
			cv2.aruco.generateImageMarker(aruco_dict, aruc_id, size, tag, 1)
			testim[y:y+size, x:x+size] = tag
			aruc_id+=1

	cv2.imshow("window", testim)
	cv2.waitKey(0) 
	"""