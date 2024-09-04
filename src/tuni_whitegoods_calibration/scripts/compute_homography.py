#!/usr/bin/env python3
 
import cv2
import numpy as np
 
if __name__ == '__main__' :
 
    # Read source image.
    im_src = cv2.imread('/home/odin-lms2/rgb_img.png', cv2.IMREAD_UNCHANGED)
    # Four corners of the book in source image
    pts_src = np.array([[612, 476], [652, 513], [703, 435],[740, 557]])

    # Four corners of the book  destination image.
    pts_dst = np.array([[450, 600],[500, 650],[550, 550],[600, 700]])
 
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