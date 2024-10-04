##Description: This message defines the result of a schedule simulation

#The simulation id that has been provided by the caller for reference
string simulation_id

#Simulation results for schedule
integration/SimulationResult aggregated_simulation_results

#Simulation results for schedule actions
integration/ActionSimulationResult[] action_simulation_results
