'''

import numpy as np
from scipy.optimize import least_squares
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


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
import matplotlib.pyplot as plt

from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates, Transform3DToPixel

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
        pose.position.x = world_point[0]
        pose.position.y = world_point[1]
        pose.position.z = world_point[2]

        # Call the from_robot_to_pixel function to get the projected pixel coordinates
        pixel = from_robot_to_pixel(pose)

        # Append the pixel coordinates (u, v) to the result list
        projected_points.append(pixel)

    return projected_points

# Function to compute the reprojection error
def reprojection_error(params, K_init, D_init, world_points, image_points):
    """Compute the reprojection error given intrinsic parameters."""
    # Extract intrinsic parameters from params
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = params
    
    # Update intrinsic matrix
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])
    
    # Update distortion coefficients
    D = np.array([k1, k2, p1, p2, k3])
    
    # Project the world points
    projected_points = project_points(K, D, world_points)
    
    # Return the error (difference between projected and actual 2D image points)
    return (projected_points - image_points).ravel()

# Example data (replace with actual calibration values)

# 3D world points (in real-world units, like meters)
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