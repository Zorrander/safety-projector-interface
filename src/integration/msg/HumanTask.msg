##Definition: Models a Human Operator Task request to AR

# Human task ID
string task_id

#used to update the status of the task execution
#Uniquely identifies the execution request to the AR
string reference_id

bool requires_acceptance

# Brief Task Description
string description

# RESOLVED, UNRESOLVED, PENDING
uint8 status

# Task Category, e.g. Inspection
string category

# Timestamp from ROS
time timestamp

#Constant Status
uint8 RESOLVED=0
uint8 UNRESOLVED=1
uint8 PENDING=2

sensor_msgs/CompressedImage resource_image
geometry_msgs/Point location
integration/OperatorInstructions operator_instructions
