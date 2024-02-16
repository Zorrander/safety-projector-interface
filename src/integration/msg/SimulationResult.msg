##Description: This message defines the common result of all simulations

Header header 

#true if the simulation shows that the schedule can be carried out
bool is_feasible

#the simulated duration of the task
uint64 duration_in_milliseconds

#the simulated (for movable robots or operators) distance in meters, for stationary robots this should be zero
float32 move_distance_in_meters

#the amount of time that human and robot spent within ranges of distances
integration/HumansRobotsSeparationDistanceDurationMessage[] humans_robots_separation_distance_duration_messages

#the max RULA-metrics encountered during the execution of this simulation's human tasks
integration/SimulationTasksMaxRulaMetrics simulation_tasks_max_rula_metrics
