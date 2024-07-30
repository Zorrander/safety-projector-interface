#!/usr/bin/env python3
 
import cv2
import numpy as np
 
if __name__ == '__main__' :
 
    # Read source image.
    im_src = cv2.imread('/home/odin3/rgb_img.png', cv2.IMREAD_UNCHANGED)
    # Four corners of the book in source image
    pts_src = np.array([[1224, 650], [1167, 598], [1101, 650],[1046, 598], [979, 650],[925, 598]])

    # Four corners of the book in destination image.
    pts_dst = np.array([[300, 300],[400, 400],[525, 300],[625, 400],[750, 300],[850, 400]])
 
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    
    print(h)
    print(status)

    # Warp source image to destination based on homography
    im_out = cv2.warpPerspective(im_src, h, (im_src.shape[1],im_src.shape[0]))
 
    # Display images
    cv2.imshow("Source Image", im_src)
    cv2.imshow("Warped Source Image", im_out)
 
    cv2.waitKey(0)