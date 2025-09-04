#!/usr/bin/env python3

import rospy

import numpy as np
from scipy.optimize import least_squares
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import math
import actionlib
from integration.msg import SetVirtualButtonsProjectionAction, SetVirtualButtonsProjectionGoal

rospy.init_node("virtual_button_client")

'''
def project_points(K, D, R, t, world_points):
    """Project 3D points onto the 2D image plane, considering distortion."""
    t = np.array(t).reshape(3, 1)  # Ensure it's a column vector
    projected_points, _ = cv2.projectPoints(world_points, R, t, K, D)
    return projected_points.squeeze()

def reprojection_error(params, K, D, world_points, image_points):
    """Compute the reprojection error given extrinsic parameters."""
    t = params[:3]  # Extract translation
    q = params[3:]  # Extract quaternion
    R_matrix = R.from_quat(q).as_matrix()  # Convert quaternion to rotation matrix
    
    projected_points = project_points(K, D, R_matrix, t, world_points)
    
    return (projected_points - image_points).ravel()

# Example data (replace with actual calibration values)

world_points = np.random.rand(10, 3) * 10  # 3D points
image_points = np.random.rand(10, 2) * 1000  # 2D image points


K = np.array([[911.1669921875, 0.0, 961.0018310546875], 
              [0.0, 911.3311157226562, 544.41552734375], 
              [0.0, 0.0, 1.0]])

D = np.array([0.5090121030807495, -2.8370401859283447, 0.00041892516310326755, 
              -9.842617873800918e-05, 1.6451900005340576, 0.3761782646179199, 
              -2.630704641342163, 1.5565040111541748])

# Example 3D world points and detected 2D image points
world_points = np.array([ [-0.40, -0.25, 0.297],
						  [-0.40, -0.0, 0.297],
						  [-0.40, 0.25, 0.295],
						  [-0.40, 0.5, 0.295], 
							 
						  [-0.575, -0.25, 0.295],
						  [-0.575, -0.0, 0.297],
						  [-0.575, 0.25, 0.295],
						  [-0.575, 0.5, 0.295],   

						  [-0.75, -0.25, 0.297],
						  [-0.75, -0.0, 0.297],
						  [-0.75, 0.25, 0.295],
						  
						])

image_points = np.array([ [515, 705], 
						  [708, 698],
	                      [900, 691],
	                      [1091, 685], 
	                      [514, 570], 
	                      [706, 564],
						  [896, 558], 
						  [1085, 552], 
						  [513, 439], 
						  [703, 432], 
						  [892, 426]])
  

# Initial guess: [tx, ty, tz, qx, qy, qz, qw]
initial_guess = [0.498229, -0.323922, 1.32144, -0.695437, 0.718008, -0.0230374, 0.017348]

# Optimize extrinsics
result = least_squares(reprojection_error, initial_guess, args=(K, D, world_points, image_points))

# Extract optimized parameters
optimized_t = result.x[:3]
optimized_q = result.x[3:]


# Normalize the quaternion
norm = np.linalg.norm(optimized_q)
normalized_q = optimized_q / norm

print("Optimized Translation:", optimized_t)
print("Optimized Quaternion:", normalized_q)

# Project the points with the optimized parameters
R_matrix = R.from_quat(normalized_q).as_matrix()
projected_points = project_points(K, D, R_matrix, optimized_t, world_points)


# Visualize reprojection


'''
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
import numpy as np
from scipy.optimize import least_squares
import cv2
import time
import matplotlib.pyplot as plt
from pathlib import Path
from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates, Transform3DToPixel, TransformRobotCameraCoordinatesRequest, Transform3DToPixelRequest


def draw_points_on_image(image, world_points):
    for world_point in world_points:
        # Convert world_point (x, y, z) to a Pose object (3D coordinates)
        pose = Pose()
        pose.position.x = world_point[0]
        pose.position.y = world_point[1]
        pose.position.z = world_point[2]

        pixel = from_robot_to_pixel(pose)
        if pixel is not None:
            u, v = int(pixel[0]), int(pixel[1])
            print(u, v)
            cv2.circle(image, (u, v), radius=5, color=(0, 0, 255), thickness=-1)  # Red filled circle
    
    return image

def from_robot_to_pixel(pose):
    # Create PoseStamped message for input point
    in_point_stamped = PoseStamped()
    in_point_stamped.header.frame_id = "base"
    in_point_stamped.header.stamp = rospy.Time(0)
    in_point_stamped.pose = pose

    # Create request to transform robot coordinates to camera coordinates
    rospy.wait_for_service('transform_world_coordinates_frame')
    try:
        client_world_coordinates = rospy.ServiceProxy('transform_world_coordinates_frame', TransformRobotCameraCoordinates)
        
        # Request to transform from robot to camera frame
        srv_pose = TransformRobotCameraCoordinatesRequest()
        srv_pose.in_point_stamped = in_point_stamped
        srv_pose.target_frame = "camera1_rgb_camera_link"

        # Call service to transform
        response_pose = client_world_coordinates(srv_pose)
        
        # Convert to 3D camera coordinates
        transformed_pose = response_pose.out_point_stamped.pose
        rospy.loginfo(f"Transformed to Camera frame: x = {transformed_pose.position.x}, y = {transformed_pose.position.y}, z = {transformed_pose.position.z}")
        
        # Project to 2D image coordinates
        rospy.wait_for_service('transform_3D_to_pixel')
        client_3D_to_pixel = rospy.ServiceProxy('transform_3D_to_pixel', Transform3DToPixel)
        
        # Request to project 3D to 2D pixel coordinates
        srv_3D_to_pixel = Transform3DToPixelRequest()
        srv_3D_to_pixel.x = transformed_pose.position.x
        srv_3D_to_pixel.y = transformed_pose.position.y
        srv_3D_to_pixel.z = transformed_pose.position.z

        # Call service to get pixel coordinates
        response_pixel = client_3D_to_pixel(srv_3D_to_pixel)
        
        # Log the projected pixel coordinates
        rospy.loginfo(f"Projected pixel coordinates: (u: {response_pixel.u}, v: {response_pixel.v})")

        # Return the pixel coordinates as a cv::Point equivalent in Python (using tuple)
        return (response_pixel.u, response_pixel.v)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def project_points(world_points):
    """Project 3D world points onto the 2D image plane using transformation services."""
    
    # Initialize the result list for the projected points
    projected_points = []

    # Loop through each 3D world point to project it to the image plane
    for world_point in world_points:
        # Convert world_point (x, y, z) to a Pose object (3D coordinates)
        pose = Pose()
        pose.position.x = world_point[0] + 0.04
        pose.position.y = world_point[1]
        pose.position.z = world_point[2]
        
        send_virtual_button("MarkerButton", pose.position.x, pose.position.y)
        # Call the from_robot_to_pixel function to get the projected pixel coordinates
        pixel = from_robot_to_pixel(pose)

        # Append the pixel coordinates (u, v) to the result list
        projected_points.append(pixel)
        time.sleep(2)
    return projected_points

from tuni_whitegoods_msgs.srv import TransformPixelToProjection, TransformPixelToProjectionRequest


def call_pixel_transform(u, v):
    rospy.wait_for_service("transform_point_to_project")
    try:
        transform_service = rospy.ServiceProxy("transform_point_to_project", TransformPixelToProjection)
        req = TransformPixelToProjectionRequest(u=u, v=v)
        res = transform_service(req)

        print(f"Input pixel: ({u}, {v})")
        print(f"Transformed pixel: ({res.u_prime:.2f}, {res.v_prime:.2f})")
        return res.u_prime, res.v_prime
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


from integration.msg import SetVirtualButtonsProjectionAction, SetVirtualButtonsProjectionGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA, Duration

def send_virtual_button(name, center_x, center_y, radius=10, lifetime_secs=0):
    client = actionlib.SimpleActionClient('/execution/projector_interface/integration/actions/set_virtual_buttons_projection', SetVirtualButtonsProjectionAction)
    rospy.loginfo("Waiting for ButtonProjectionServer...")
    client.wait_for_server()

    goal = SetVirtualButtonsProjectionGoal()
    goal.zone = "table"

    # Define the virtual button
    button = goal.virtual_button
    button.name = name
    button.text = ""
    button.description = ""
    button.id = name
    button.zone = goal.zone

    # Colors
    button.button_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    button.text_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

    # Center as Pose
    button.center = Pose()
    button.center.position.x = center_x
    button.center.position.y = center_y
    button.center.position.z = 0.0
    button.center.orientation.x = 0.0
    button.center.orientation.y = 0.0
    button.center.orientation.z = 0.0
    button.center.orientation.w = 1.0

    # Other fields
    button.radius = radius
    button.lifetime.secs = lifetime_secs
    button.lifetime.nsecs = 0
    button.hidden = False

    rospy.loginfo(f"Sending virtual button '{name}' at ({center_x}, {center_y})")
    client.send_goal(goal)

# Function to compute the reprojection error
def reprojection_error(world_points):
    """Compute the reprojection error given intrinsic parameters."""
    
    # Project the world points
    projected_points = project_points(world_points)

    print(projected_points)
    image_points = np.array([ [555, 626], [739, 619], [918, 612], [1096, 606], [555, 500], [738, 495], [916, 490], [1092, 485], [554, 375], [736, 355]])
    return (projected_points - image_points).ravel()

# store points
points = []


def select_point(event, x, y, flags, param):
    global points
    img = param
    if event == cv2.EVENT_LBUTTONDBLCLK:  # left button double-

        points.append((x, y))
        print(f"Point selected: ({x}, {y})")
        
        # Draw a small circle where user clicked
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow('Points', img)

        # If two points selected, calculate distance
        if len(points) == 2:
            p1, p2 = points
            distance = math.dist(p1, p2)  # Python 3.8+ (Euclidean distance)
            print(f"Distance between {p1} and {p2}: {distance:.2f} pixels")
            
            # Draw a line between the points
            cv2.line(img, p1, p2, (255, 0, 0), 2)
            cv2.imshow('Points', img)

            # reset for next measurement
            points = []

# Example data (replace with actual calibration values)

# 3D world points (in real-world units, like meters)

'''
world_points = np.array([ [-0.40, -0.25, 0.297],
                          [-0.40, -0.0, 0.297],
                          [-0.40, 0.25, 0.295],
                          [-0.40, 0.5, 0.295], 
                          [-0.575, -0.25, 0.295],
                          [-0.575, -0.0, 0.297],
                          [-0.575, 0.25, 0.295],
                          [-0.575, 0.5, 0.295],   
                          [-0.75, -0.25, 0.297],
                          [-0.75, -0.0, 0.297],
                          [-0.75, 0.25, 0.295]])

# 2D image points (corresponding to the world points)
image_points = np.array([ [515, 705], 
                          [708, 698],
                          [900, 691],
                          [1091, 685], 
                          [514, 570], 
                          [706, 564],
                          [896, 558], 
                          [1085, 552], 
                          [513, 439], 
                          [703, 432], 
                          [892, 426]])

# Initial guess for intrinsic parameters: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
initial_guess = [911.1669921875, 911.3311157226562, 961.0018310546875, 544.41552734375, 
                 0.5090121030807495, -2.8370401859283447, 0.00041892516310326755, 
                 -9.842617873800918e-05, 1.6451900005340576]

# Perform the optimization using least squares
result = least_squares(reprojection_error, initial_guess, args=(None, None, world_points, image_points))

# Extract the optimized intrinsic parameters
optimized_params = result.x
fx_opt, fy_opt, cx_opt, cy_opt, k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = optimized_params

print("Optimized Intrinsic Parameters:")
print(f"Focal Length (fx, fy): ({fx_opt}, {fy_opt})")
print(f"Principal Point (cx, cy): ({cx_opt}, {cy_opt})")
print(f"Distortion Coefficients (k1, k2, p1, p2, k3): ({k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}, {k3_opt})")

# Project the points with the optimized parameters

projected_points = project_points(K, D, R_matrix, optimized_t, world_points)

plt.figure(figsize=(8, 6))
plt.scatter(image_points[:, 0], image_points[:, 1], color='red', label='Observed Points')
plt.scatter(projected_points[:, 0], projected_points[:, 1], color='blue', label='Projected Points')
plt.legend()
plt.title("Reprojection of 3D Points")
plt.xlabel("X Pixel")
plt.ylabel("Y Pixel")
plt.show()
'''
#image_path = Path.home() / 'data_calib' / 'rectrgb_img.png'
image_path = Path.home() / 'data_calib' / 'rawrgb_img.png'

# Read the image
frame = cv2.imread(str(image_path))

world_points = np.array([ [0.45, 0.225, 0.0],
                          [0.45, -0.025, 0.005],
                          [0.45, -0.275, 0.005],
                          [0.45, -0.525, 0.005], 
                          [0.625, 0.225, 0.005],
                          [0.625, -0.025, 0.005],
                          [0.625, -0.275, 0.005],
                          [0.625, -0.525, 0.005],   
                          [0.80, 0.225, 0.005],
                          [0.825, -0.025, 0.005]])

frame_with_points = draw_points_on_image(frame, world_points)
print(reprojection_error(world_points))

cv2.namedWindow('Points')
cv2.setMouseCallback('Points', select_point, frame)
cv2.imshow("Points", frame_with_points)
cv2.waitKey(0)
cv2.destroyAllWindows()


'''

Point selected: (572, 401)
Point selected: (555, 375)
Distance between (572, 401) and (555, 375): 31.06 pixels
Point selected: (742, 379)
Point selected: (736, 354)
Distance between (742, 379) and (736, 354): 25.71 pixels
Point selected: (571, 521)
Point selected: (555, 501)
Distance between (571, 521) and (555, 501): 25.61 pixels
Point selected: (745, 515)
Point selected: (738, 496)
Distance between (745, 515) and (738, 496): 20.25 pixels
Point selected: (917, 510)
Point selected: (916, 490)
Distance between (917, 510) and (916, 490): 20.02 pixels
Point selected: (1090, 504)
Point selected: (1093, 485)
Distance between (1090, 504) and (1093, 485): 19.24 pixels
Point selected: (1094, 625)
Point selected: (1096, 606)
Distance between (1094, 625) and (1096, 606): 19.10 pixels
Point selected: (921, 631)
Point selected: (918, 611)
Distance between (921, 631) and (918, 611): 20.22 pixels
Point selected: (747, 635)
Point selected: (740, 618)
Distance between (747, 635) and (740, 618): 18.38 pixels
Point selected: (574, 642)
Point selected: (555, 626)
Distance between (574, 642) and (555, 626): 24.84 pixels





Distance between (564, 399) and (562, 378): 21.10 pixels
Point selected: (741, 378)
Point selected: (738, 356)
Distance between (741, 378) and (738, 356): 22.20 pixels
Point selected: (917, 510)
Point selected: (916, 490)
Distance between (917, 510) and (916, 490): 20.02 pixels
Point selected: (744, 515)
Point selected: (739, 495)
Distance between (744, 515) and (739, 495): 20.62 pixels
Point selected: (1090, 504)
Point selected: (1092, 485)
Distance between (1090, 504) and (1092, 485): 19.10 pixels
Point selected: (1095, 625)
Point selected: (1096, 605)
Distance between (1095, 625) and (1096, 605): 20.02 pixels
Point selected: (921, 631)
Point selected: (918, 611)
Distance between (921, 631) and (918, 611): 20.22 pixels
Point selected: (746, 637)
Point selected: (741, 618)
Distance between (746, 637) and (741, 618): 19.65 pixels
Point selected: (567, 644)
Point selected: (562, 624)
Distance between (567, 644) and (562, 624): 20.62 pixels
Point selected: (566, 521)
Point selected: (562, 500)
Distance between (566, 521) and (562, 500): 21.38 pixels



[(601, 640), (775, 634), (949, 629), (1122, 623), (599, 518), (772, 513), (945, 508), (1117, 502), (598, 399), (769, 377)]
[46 14 36 15 31 17 26 17 44 18 34 18 29 18 25 17 44 24 33 22]
Point selected: (570, 401)
Point selected: (589, 394)
Distance between (570, 401) and (589, 394): 20.25 pixels
Point selected: (742, 379)
Point selected: (761, 371)
Distance between (742, 379) and (761, 371): 20.62 pixels
Point selected: (571, 520)
Point selected: (587, 516)
Distance between (571, 520) and (587, 516): 16.49 pixels
Point selected: (745, 515)
Point selected: (764, 511)
Distance between (745, 515) and (764, 511): 19.42 pixels
Point selected: (916, 510)
Point selected: (939, 507)
Distance between (916, 510) and (939, 507): 23.19 pixels
Point selected: (1090, 504)
Point selected: (1111, 499)
Distance between (1090, 504) and (1111, 499): 21.59 pixels
Point selected: (1094, 625)
Point selected: (1117, 623)
Distance between (1094, 625) and (1117, 623): 23.09 pixels
Point selected: (921, 631)
Point selected: (943, 629)
Distance between (921, 631) and (943, 629): 22.09 pixels
Point selected: (574, 642)
Point selected: (591, 639)
Distance between (574, 642) and (591, 639): 17.26 pixels

'''