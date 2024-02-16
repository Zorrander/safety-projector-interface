##Usage: Define the various statuses of actions

##Constants
#Actions Status for Feedback
string IN_PROGRESS=InProgress

#Actions Status for Result
string SUCCESS=Success 
#Action terminated successfully

string EMERGENCY_STOP=EmergencyStop 
#Action terminated due to an emergency stop

string SAFETY_ZONE_VIOLATION=SafetyZoneViolation 
#Action terminated due to a safety zone violation

string OPERATIONAL_EXCEPTION=OperationalException 
#Action terminated due to an operational exception

string MALFUNCTION=Malfunction 
#Action terminated due to malfunction before being successfully completed

string CANCELLED=Cancelled 
#Action cancelled before being successfully completed

string PAUSED=Paused 
#Action paused before being successfully completed.