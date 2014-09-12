README - EXPERIMENTS AND LOGGING

This simulator has been developed for the Pi-Swarm robots at the University of York.
It is written in Python and utilises Pybox2D and Pygame.
Development of the initial version by Becky Naylor funded by the BPSI.

Running Experiments

To run an experiment using the simulator use the command:
"python Pi-Swarm-Sim.py --experiment True"

The simulator reads parameters from a file named ''experiment_settings.txt'' in the same directory as Pi-Swarm-Sim.py. The first line should be the name of your experiment. The second line is a dictionary of dictionaries each of which contains the parameters for a single robot. Currently only the 'startBatteryLevel' parameter is set (but it is simple to change the code in robot.py to set any other parameter, just copy the if statement used for 'startbatterylevel').

By default the seed is set to the time when the experiment is run. It is written to the log file for replication. It is possible to run a set of experiments using the same seed by fixing the seed in Pi-Swarm-Sim.py. 

Log files

Each experiment produces a directory with the experiment name given in ''experiment_settings.txt''. Inside this directory there will be: 

1. A copy of the original 'experiment_settings.txt' file.
2. one file per robot named 'robotdataX.log' where X is the robot id number. This contains the timestep, position, heading, state, battery level out of 2400 and robot's neighbours. 
3. settings.log which contains the parameter settings used for the experiment.
4. Simulation.log gives overall status for each timestep. The timestep, number of unexplored tiles, % unexplored and total power.
5. If at least one of the robots has stopped due to running out of power then there will be a finish-times.log which contains the time at which each robot stopped.

Pi-Swarm-Sim.py can be updated so that the logs produce a different output should it be required. 
