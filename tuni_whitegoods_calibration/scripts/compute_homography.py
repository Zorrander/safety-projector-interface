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
    pts_src = np.array([[686, 222], [718, 222], [748, 223],[684, 249], [715, 248],[748, 249], [682, 276],[714, 277], [746, 277], 
                        [1003, 287], [1035, 287], [1066, 288],[1004, 316], [1036, 315],[1069, 315], [1005, 344],[1038, 344], [1071, 343]])
    # Four corners of the book  destination image.
    #pts_dst = np.array([[1401, 599],[1376, 233],[1645, 220],[1675, 592]])
    pts_dst = np.array([[500, 250],[540, 250],[580, 250],[500, 290],[540, 290],[580, 290],[500, 330],[540, 330],[580, 330],
                        [900, 350],[940, 350],[980, 350],[900, 390],[940, 390],[980, 390],[900, 430],[940, 430],[980, 430]])
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    print(h)
    print(status)


if __name__ == '__main__' :

    compute_moving_table_homography()
