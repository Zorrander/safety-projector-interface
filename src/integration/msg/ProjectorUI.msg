##Description: Describes a predefined UI. 

string resource_id #A unique identifier
string zone        #where to display this UI
string name #Human readable name
string description #Human readable description
bool hidden        #if this interface is hidden e.g available for later use (dynamically switch between interface)
string[] configuration_ids #The available configurations for this UI. E.g. TopLeft, TopRight
integration/VirtualButtonReference[] virtual_button_references #The references of buttons that are included in this predefined UI.
