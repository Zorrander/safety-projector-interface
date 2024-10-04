##Description: Describes a virtual button. 
string id #A unique identifier. Uniquely identifies this virtual button. Used to identify the input.
string zone                      #In which zone would the button be displayed
string name #Human readable name
string description #Human readable description
string text #Textual description of the option
std_msgs/ColorRGBA button_color # Color [0.0-1.0]
std_msgs/ColorRGBA text_color    # Color [0.0-1.0]
geometry_msgs/Pose center	#where to display the buttons
float32 radius	#size of buttons
duration lifetime # How long this button will be shown before being automatically deleted.  0 means forever
bool hidden #If true, then this describes a signal coming from a non-visible button.
