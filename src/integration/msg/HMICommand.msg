##Usage: Channel for HMI requests e.g. Safety Reset, Start Schedule, Pause Resume execution
##Description:
string operator_id ##optional
string module_id ##Required
string command ## Required 
string message ##Used only for debug

string RESET_SAFETY_COMMAND=ResetSafety
string RESUME_SCHEDULE_COMMAND=ResumeSchedule
string PAUSE_SCHEDULE_COMMAND=PauseSchedule