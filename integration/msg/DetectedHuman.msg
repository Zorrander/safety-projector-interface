##Description: A human skeleton model
# header for time/frame information
Header header

#The reference frame of the 
geometry_msgs/TransformStamped reference_frame

#Used to identify multiple detections
string detected_human_id

#A set of operator joints poses. The Openni_tracker model is used https://wiki.ros.org/openni_tracker
#Should use as a reference the same frame, provided in reference_frame
geometry_msgs/PoseStamped head
geometry_msgs/PoseStamped neck
geometry_msgs/PoseStamped torso
geometry_msgs/PoseStamped left_shoulder
geometry_msgs/PoseStamped left_elbow
geometry_msgs/PoseStamped left_hand
geometry_msgs/PoseStamped right_shoulder
geometry_msgs/PoseStamped right_elbow
geometry_msgs/PoseStamped right_hand
geometry_msgs/PoseStamped left_hip
geometry_msgs/PoseStamped left_knee
geometry_msgs/PoseStamped left_foot
geometry_msgs/PoseStamped right_hip
geometry_msgs/PoseStamped right_knee
geometry_msgs/PoseStamped right_foot