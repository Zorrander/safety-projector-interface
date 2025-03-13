#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Camera intrinsic parameters (replace with actual calibration data)
#CAMERA_MATRIX = np.array([[944.055330, 0.0, 960.777526], [0.0, 947.544800, 556.218561], [0.0, 0.0, 1.0]])  # Example
#DIST_COEFFS = np.array([0.094592, -0.027569, 0.002634, -0.000245, 0.000000])  # Example distortion coefficients


#CAMERA_MATRIX = np.array([[935.602589, 0.000000, 959.302052], [0.000000, 939.038616, 544.892833], [0.0, 0.0, 1.0]])  # Example
#DIST_COEFFS = np.array([0.087177, -0.036716, -0.001525, 0.000284, 0.000000])  # Example distortion coefficients

CAMERA_MATRIX = np.array([[911.1669921875, 0.0, 961.0018310546875], [0.0, 911.3311157226562, 544.41552734375], [0.0, 0.0, 1.0]])  # Example
DIST_COEFFS = np.array([0.5090121030807495, -2.8370401859283447, 0.00041892516310326755, -9.842617873800918e-05, 1.6451900005340576, 0.3761782646179199, -2.630704641342163, 1.5565040111541748])  # Example distortion coefficients

# Chessboard pattern size (adjust to your checkerboard)
CHESSBOARD_SIZE = (14, 7)  # (width, height) - internal corners

# Initialize bridge for ROS image conversion
bridge = CvBridge()

def distortion_evaluation(msg):
    """Callback function to process incoming ROS image and evaluate distortion."""
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if ret:
            # Refine corner positions
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                       criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            # Undistort detected points
            undistorted_points = cv2.undistortPoints(corners, CAMERA_MATRIX, DIST_COEFFS, P=CAMERA_MATRIX)

            # Compute distortion residuals (Euclidean distance)
            errors = np.linalg.norm(corners.squeeze() - undistorted_points.squeeze(), axis=1)

            # Compute mean and max residual error
            mean_error = np.mean(errors)
            max_error = np.max(errors)

            rospy.loginfo(f"Mean Distortion Residual: {mean_error:.4f} pixels")
            rospy.loginfo(f"Max Distortion Residual: {max_error:.4f} pixels")

            # Draw corners on the image
            cv2.drawChessboardCorners(cv_image, CHESSBOARD_SIZE, corners, ret)
            cv2.imshow("Detected Chessboard", cv_image)
            cv2.waitKey(1)
        else:
            rospy.logwarn("Chessboard not found in the current frame!")

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def line_intersection(rho, theta, width, height):
    """
    Calculate the intersection points of the line (rho, theta) with the image borders.
    """
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    return (x1, y1), (x2, y2)

def find_matching_lines(lines_original, lines_undistorted, threshold_rho=5, threshold_theta=0.1):
    """
    Find lines in the original and undistorted images that match by comparing their (rho, theta).
    """
    matched_lines = []
    
    for line_orig in lines_original:
        rho_orig, theta_orig = line_orig[0]
        
        closest_match = None
        min_distance = float('inf')
        
        for line_undist in lines_undistorted:
            rho_undist, theta_undist = line_undist[0]
            
            # Calculate the distance between the lines in the parameter space
            rho_diff = np.abs(rho_orig - rho_undist)
            theta_diff = np.abs(theta_orig - theta_undist)
            
            if rho_diff < threshold_rho and theta_diff < threshold_theta:
                distance = rho_diff + theta_diff
                if distance < min_distance:
                    min_distance = distance
                    closest_match = line_undist
        
        if closest_match is not None:
            matched_lines.append((line_orig, closest_match))
    
    return matched_lines

def image_callback(msg):
    bridge = CvBridge()

    # Convert the ROS Image message to OpenCV format
    try:
        original_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr("Failed to convert image: %s", str(e))
        return

    undistorted_image = cv2.undistort(original_image, CAMERA_MATRIX, DIST_COEFFS)

    # Convert both images to grayscale for edge detection
    gray_original = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)
    gray_undistorted = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

    # Apply edge detection (Canny)
    edges_original = cv2.Canny(gray_original, 50, 150, apertureSize=3)
    edges_undistorted = cv2.Canny(gray_undistorted, 50, 150, apertureSize=3)

    # Detect lines using Hough Line Transform
    lines_original = cv2.HoughLines(edges_original, 1, np.pi / 180, threshold=300)
    lines_undistorted = cv2.HoughLines(edges_undistorted, 1, np.pi / 180, threshold=300)

    # Find matching lines between the original and undistorted images
    matched_lines = find_matching_lines(lines_original, lines_undistorted)

    # Initialize a list to store line shifts
    line_shifts = []

    # Iterate over the matched lines and calculate the shift
    for orig_line, undist_line in matched_lines:
        rho_orig, theta_orig = orig_line[0]
        rho_undist, theta_undist = undist_line[0]

        # Get the intersection points of the original and undistorted lines with the image borders
        orig_intersect = line_intersection(rho_orig, theta_orig, original_image.shape[1], original_image.shape[0])
        undist_intersect = line_intersection(rho_undist, theta_undist, undistorted_image.shape[1], undistorted_image.shape[0])

        # Calculate the Euclidean distance between the intersection points (shift)
        shift = np.linalg.norm(np.array(orig_intersect[0]) - np.array(undist_intersect[0]))
        line_shifts.append(shift)

        # Visualize the lines
        cv2.line(original_image, orig_intersect[0], orig_intersect[1], (0, 0, 255), 2)  # Red lines in original image
        cv2.line(undistorted_image, undist_intersect[0], undist_intersect[1], (0, 255, 0), 2)  # Green lines in undistorted image

        # Optionally, draw the shift (deviation)
        cv2.circle(original_image, orig_intersect[0], 10, (0, 0, 255), -1)  # Mark the intersection in red
        cv2.circle(undistorted_image, undist_intersect[0], 10, (0, 255, 0), -1)  # Mark the intersection in green

    # Calculate and display the mean line shift (deviation)
    mean_shift = np.mean(line_shifts)
    rospy.loginfo(f"Mean Line Shift (Deviation): {mean_shift:.2f} pixels")

    # Display the results
    cv2.imshow("Original Image with Lines", original_image)
    cv2.imshow("Undistorted Image with Lines", undistorted_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('line_distortion_evaluator', anonymous=True)

    # Set up the subscriber to the rgb/image_raw topic
    rospy.Subscriber("/rgb/image_rect_color", Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
    '''
    header: 
      seq: 553
      stamp: 
        secs: 1741797009
        nsecs: 349234086
      frame_id: "rgb_camera_link"
    height: 1080
    width: 1920
    distortion_model: "rational_polynomial"
    D: [0.5090121030807495, -2.8370401859283447, 0.00041892516310326755, -9.842617873800918e-05, 1.6451900005340576, 0.3761782646179199, -2.630704641342163, 1.5565040111541748]
    K: [911.1669921875, 0.0, 961.0018310546875, 0.0, 911.3311157226562, 544.41552734375, 0.0, 0.0, 1.0]
    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P: [911.1669921875, 0.0, 961.0018310546875, 0.0, 0.0, 911.3311157226562, 544.41552734375, 0.0, 0.0, 0.0, 1.0, 0.0]
    binning_x: 0
    binning_y: 0
    roi: 
      x_offset: 0
      y_offset: 0
      height: 0
      width: 0
      do_rectify: False
    '''