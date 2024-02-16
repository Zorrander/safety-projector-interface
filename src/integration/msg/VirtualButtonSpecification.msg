integration/VirtualButtonReference virtual_button_reference #includes ID

string text #Textual description of the option
std_msgs/ColorRGBA button_color # Color [0.0-1.0]
geometry_msgs/Pose center	#where to display the buttons
float32 radius	#size of buttons
duration lifetime # How long this button will be shown before being automatically deleted.  0 means forever


