string virtual_button_id #uniquely identifies the virtual button pressed.
int32 event_type # Determine if the button has been pressed, or has been released. Anything else is considered invalid
int32 PRESSED  = -1 #Button is being pressed down
int32 RELEASED = +1 #Button has been released after being pressed down.