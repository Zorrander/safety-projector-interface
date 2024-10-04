##Description: This message defines the common result of all simulations

Header header 

bool requestActionSimulationResults #if true detailed, action level results will be provided
#The simulation results
integration/SimulationResourceStatus[] simulationResourcesFinalStatus
integration/ScheduleSimulationResult scheduleSimulationResult 

#If true means that the simulation is in progress, if false means that the operation is finished, or not started.
bool simulation_in_progress

#A true value indicates an error, false means error was detected 
#If error_in_simulation==true and simulation_in_progress==false then this is an error signal
bool error_in_simulation