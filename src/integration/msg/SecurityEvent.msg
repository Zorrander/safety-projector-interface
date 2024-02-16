##Description: Describes a security related event
Header header

string id #Uniquely identifies this event. Provided for future reference


string type #The type of the event.
string sender_id #An id that is used to identify the sender of this security_event.
string network_id #Optional an id that defines the id of a particular network segment
int32 severity_level #Used to define the severity of the event the higher the more severe. Use only positive integers
