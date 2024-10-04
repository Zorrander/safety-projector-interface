##OperatorInstructions ROS message
##Usage: In ExecuteHumanTask.action
##Definition: Models the instructions for a single operator task.

string id
string name
string title
string description
string subassembly_group
string part_id
string tool_id
int32 assistance_level
int32 task_order
integration/AssemblyAnimation[] assembly_animations


###Note: An Operator Instruction should contain either a location id OR a specific tagLocation not both.
###An operator instruction can also contain neither a location id nor a specific tagLocation.
###If both are present the behaviour is not defined.

##Location Id to show the tag.
string location_id_for_tag

##Location to show the tag
geometry_msgs/PoseStamped tag_location