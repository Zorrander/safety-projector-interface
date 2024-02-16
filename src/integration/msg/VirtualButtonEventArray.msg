#Description: Describes virtual button events that happened at the same time.
#Publishing a message where virtal_button_events is empty should be considered invalid.
#Multiple events in the same message indicate that they have taken place simultaneously. E.g. Up and left button have been pressed together.

Header header
integration/VirtualButtonEvent[] virtual_button_events
