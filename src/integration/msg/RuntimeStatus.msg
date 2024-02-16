##Description: Describes the runtime status of an identifiable, named object
string id
string name
string status

##Can be null if not started
time start_time

##Can be null if not ended
time end_time

##From 0 to 100 defines an estimate of the percentage of completion of the item.
##If <0 it means that an estimate cannot be provided.
int32 completion_percentage