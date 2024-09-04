#!/usr/bin/env python3

import cv2
import numpy as np

# Load the depthmap
depthmap = cv2.imread('/home/odin-lms2/depthmap.png', cv2.IMREAD_UNCHANGED)

# Normalize the depth map to the range 0-255 for better visualization
depth_normalized = cv2.normalize(depthmap, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

# Apply a color map
depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

# Display the depthmap and select ROI
roi = cv2.selectROI("Select ROI", depth_colormap)
x1, y1, w, h = roi
color_roi_depthmap = depth_colormap[y1:y1+h, x1:x1+w]
roi_depthmap = depthmap[y1:y1+h, x1:x1+w]
print(x1, y1, w, h)

# Calculate the mean and standard deviation within the ROI
average_depth = np.mean(roi_depthmap)
std_dev_depth = np.std(roi_depthmap)

# Define depth_min and depth_max based on the mean and standard deviation
depth_min = max(0, average_depth - std_dev_depth)
depth_max = min(np.max(roi_depthmap), average_depth + std_dev_depth)

print(f"Average depth value within the ROI: {average_depth}mm")
print(f"Depth min: {depth_min}mm")
print(f"Depth max: {depth_max}mm")

# Display the selected ROI
cv2.imshow("Selected ROI", roi_depthmap)
cv2.waitKey(0)
cv2.destroyAllWindows()