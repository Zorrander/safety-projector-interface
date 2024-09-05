#!/usr/bin/env python3

import rospy
import cv2

# stores mouse position in global variables ix(for x coordinate) and iy(for y coordinate) 
# on double click inside the image
def select_point(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK: # captures left button double-click
        print(x,y)

img = cv2.imread('/home/odin-lms2/rgb_img.png', cv2.IMREAD_UNCHANGED)
cv2.namedWindow('image')
# bind select_point function to a window that will capture the mouse click
cv2.setMouseCallback('image', select_point)
cv2.imshow('image',img)
cv2.waitKey(0)   
cv2.destroyAllWindows()