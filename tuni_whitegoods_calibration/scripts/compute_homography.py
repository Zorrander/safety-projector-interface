#!/usr/bin/env python3
 
import cv2
import numpy as np


def compute_table_homography():
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


def compute_moving_table_homography():
    # Four corners of the book in source image
    #pts_src = np.array([[1399, 596], [1394, 505], [1488, 503],[1494, 595]])
    pts_src = np.array([[1294, 375], [1202, 380], [1200, 293],[1289, 290]])
    # Four corners of the book  destination image.
    #pts_dst = np.array([[1401, 599],[1376, 233],[1645, 220],[1675, 592]])
    pts_dst = np.array([[1294, 375],[1098, 379],[1090, 250],[1287, 240]])
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    print(h)
    print(status)


if __name__ == '__main__' :

    compute_moving_table_homography()
