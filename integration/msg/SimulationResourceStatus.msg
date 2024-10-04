#Description: Defines the initialization state of a resource for the simulation.

string resource_id
string resource_type ##RESOURCE_TYPE_OPERATOR or RESOURCE_TYPE_STATIONARY_ROBOT or or RESOURCE_TYPE_MOVABLE_ROBOT
geometry_msgs/Pose resource_initial_position

#A JointState per arm. In the case of operators this is empty.
sensor_msgs/JointState[] resource_initial_states

string RESOURCE_TYPE_OPERATOR=operator
string RESOURCE_TYPE_MOVABLE_ROBOT=stationary_robot
string RESOURCE_TYPE_STATIONARY_ROBOT=movable_robot
string RESOURCE_TYPE_STATIONARY_OBJECT=stationary_object
string RESOURCE_TYPE_MOVABLE_OBJECT=movable_object
