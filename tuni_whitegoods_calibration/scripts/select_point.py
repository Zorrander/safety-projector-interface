#!/usr/bin/env python3

import rospy
import cv2
from pathlib import Path



# stores mouse position in global variables ix(for x coordinate) and iy(for y coordinate) 
# on double click inside the image
def select_point(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK: # captures left button double-click
        print(x,y)
        print(depthmap[y, x])

img = cv2.imread(str(Path.home() / 'rgb_img.png'), cv2.IMREAD_UNCHANGED)
depthmap = cv2.imread(str(Path.home() / 'depthmap.png'), cv2.IMREAD_UNCHANGED)

cv2.namedWindow('image')
# bind select_point function to a window that will capture the mouse click
cv2.setMouseCallback('image', select_point)
cv2.imshow('image',img)
cv2.waitKey(0)   
cv2.destroyAllWindows()