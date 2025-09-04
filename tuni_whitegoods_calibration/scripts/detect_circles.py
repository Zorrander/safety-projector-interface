#!/usr/bin/env python3

import sys
import cv2 as cv
import numpy as np
import math

# font
font = cv.FONT_HERSHEY_SIMPLEX

# fontScale
fontScale = 1
 
# Blue color in BGR
color = (255, 0, 0)

# Line thickness of 2 px
thickness = 2


def compute_reprojection_error(original_pts, reprojected_pts):
    """
    Computes the reprojection error given two sets of corresponding points.

    Parameters:
    - original_pts: np.array of shape (N, 2) or (N, 3), original points
    - reprojected_pts: np.array of shape (N, 2) or (N, 3), reprojected points

    Returns:
    - mean_error: Mean Euclidean distance between the points
    - errors: List of individual errors
    """
    errors = np.linalg.norm(original_pts - reprojected_pts, axis=1)  # Euclidean distance
    mean_error = np.mean(errors)
    return mean_error, errors



def detect(path):
    filename = path
    # Loads an image
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    
    # Blur using 3 * 3 kernel. 
    # gray_blurred = cv.blur(gray, (3, 3)) 
    
    gray = cv.medianBlur(gray, 5)
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 16,
                               param1=60, param2=20,
                               minRadius=20, maxRadius=30)
    
    original_pts = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle_id, coords in enumerate(circles[0]):
            center = (coords[0], coords[1])
            original_pts.append((center, coords[2]))
            # circle center
            cv.circle(src, center, 1, (0, 100, 100), 3)
            image = cv.putText(src, str(circle_id), center, font, 
                   fontScale, color, thickness, cv.LINE_AA)
            # circle outline
            radius = coords[2]
            cv.circle(src, center, radius, (255, 0, 255), 3)
    
    
    cv.imshow("detected circles", src)
    cv.waitKey(0)
    return src, original_pts

def sort(center_i, center_j, filename):
    img = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
    sorted_org = []
    sorted_centers = []
    for center in center_i:
        closest_center = center_j[0]
        min_distance = math.dist(center[0], closest_center[0])
        for matching_center in center_j[1:]:
            if math.dist(center[0], matching_center[0]) < min_distance:
                min_distance = math.dist(center[0], matching_center[0])
                closest_center = matching_center
        if min_distance < 20:
            sorted_org.append(center)
            sorted_centers.append(closest_center)
        else:
            print("center was deleted because not match was found")
    print("""

    Sorted centers
        """)
    print(sorted_org)
    print("--------------")
    print(sorted_centers)
    print("""

    -------------------
        """)
    for circle_id, coords in enumerate(sorted_org):
        center = (coords[0][0], coords[0][1])
        center_txt = (coords[0][0]+20, coords[0][1]+20)
        # circle center
        cv.circle(img, center, 1, (0, 100, 100), 3)
        image = cv.putText(img, str(circle_id), center_txt, font, 
               fontScale, color, thickness, cv.LINE_AA)
        # circle outline
        radius = coords[1]
        cv.circle(img, center, radius, (255, 0, 255), 3)
    cv.imshow("sorted circles", img)
    cv.waitKey(0)

    for circle_id, coords in enumerate(sorted_centers):
        center = (coords[0][0], coords[0][1])
        center_txt = (coords[0][0]+30, coords[0][1]+30)

        # circle center
        cv.circle(img, center, 1, (0, 100, 100), 3)
        image = cv.putText(img, str(circle_id), center_txt, font, 
               fontScale, color, thickness, cv.LINE_AA)
        # circle outline
        radius = coords[1]
        cv.circle(img, center, radius, (255, 0, 255), 3)
    cv.imshow("sorted circles", img)
    cv.waitKey(0)
    return sorted_org, sorted_centers

def main(argv):
    #detect('/home/odin3/Documents/calibration_data/robot-cam/first/compare.png')
    img, original_pts = detect('/home/odin3/data_calib/1/drawings/rectrgb_img.png')
    img, reprojected_pts = detect('/home/odin3/data_calib/1/offset/rectrgb_img.png')
    #img, reprojected_pts = detect('/home/odin3/catkin_ws/left0000.jpg')
    original_pts, reprojected_pts = sort(original_pts, reprojected_pts, '/home/odin3/data_calib/1/drawings/rectrgb_img.png')
    
    original_pts = [x[0] for x in original_pts]
    reprojected_pts = [x[0] for x in reprojected_pts]

    original_pts = np.array(original_pts, dtype=np.float32)
    reprojected_pts = np.array(reprojected_pts, dtype=np.float32)
    
    mean_error, errors = compute_reprojection_error(original_pts, reprojected_pts)
    print(f"Mean Reprojection Error: {mean_error:.4f}")
    print(f"Individual Errors: {errors}")


    '''
    
    default_file = '/home/odin3/Documents/calibration_data/run1/circle.png'
    filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    
    # Blur using 3 * 3 kernel. 
    # gray_blurred = cv.blur(gray, (3, 3)) 
    
    gray = cv.medianBlur(gray, 5)
    
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 16,
                               param1=100, param2=30,
                               minRadius=25, maxRadius=30)
    
    original_pts = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            original_pts.append(center)
            # circle center
            cv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(src, center, radius, (255, 0, 255), 3)
    
    
    cv.imshow("detected circles", src)
    cv.waitKey(0)


    file2 = '/home/odin3/Documents/calibration_data/run1/only_projections.png'

    # Loads an image
    src = cv.imread(cv.samples.findFile(file2), cv.IMREAD_COLOR)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    
    # Blur using 3 * 3 kernel. 
    # gray_blurred = cv.blur(gray, (3, 3)) 
    
    gray = cv.medianBlur(gray, 5)
    
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 16,
                               param1=100, param2=30,
                               minRadius=20, maxRadius=25)
    
    reprojected_pts = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            reprojected_pts.append(center)
            # circle center
            cv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(src, center, radius, (255, 0, 255), 3)
    
    
    cv.imshow("detected circles", src)
    cv.waitKey(0)

    original_pts = np.array(original_pts, dtype=np.float32)
    reprojected_pts = np.array(reprojected_pts, dtype=np.float32)
    
    mean_error, errors = compute_reprojection_error(original_pts, reprojected_pts)
    print(f"Mean Reprojection Error: {mean_error:.4f}")
    print(f"Individual Errors: {errors}")
    
    return 0

    '''
if __name__ == "__main__":
    main(sys.argv[1:])