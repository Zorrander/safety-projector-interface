#!/usr/bin/env python3

import cv2
import numpy as np
from pathlib import Path
from cv2 import aruco

# Load the image and the ArUco dictionary
copied_image = cv2.imread(str(Path.home() / 'rgb_img.png'), cv2.IMREAD_UNCHANGED)
gray_image = cv2.cvtColor(copied_image, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Detect the markers in the image
marker_corners, marker_ids, rejected_candidates = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

pts_src = []
pts_dst = []

# Check if any markers are detected
if marker_ids is not None:
    for i, marker_id in enumerate(marker_ids):
        # Draw circles around the detected marker corners (red circles)
        for corner in marker_corners[i][0]:
            # Convert the floating-point corner coordinates to integers
            center = tuple(map(int, corner))  # Convert each corner (x, y) to integers
            # Draw circles on the detected marker corners
            cv2.circle(copied_image, center, 10, (0, 0, 255), 2)

        print(marker_corners[i])

# Display the image with drawn markers
cv2.imshow('Detected Markers', copied_image)
cv2.waitKey(0)
cv2.destroyAllWindows()