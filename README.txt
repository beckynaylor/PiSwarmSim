README

This simulator has been developed for the Pi-Swarm robots at the University of York.
It is written in Python and utilises Pybox2D and Pygame.
Development of the initial version by Becky Naylor funded by the BPSI.

Installation instructions are given in 'Installation Instructions.pdf'
Assumptions that the simulator uses are outlined in 'Assumptions and Settings.doc'
Plans for future development are given in 'Future Extensions.doc'

Current Operation

The robots explore the environment and head back to recharge once their battery level has reduced to below a given threshold. 

Simulator structure

1. The main simulator file is Pi-Swarm-Sim.py it sets up the simulation.

The settings such as number of robots and obstacles are set in this file in runSim under the comment '#Define simulation values - these can be altered to suit your experiment'. 

'Step' controls the simulation behaviour at each timestep and 'Draw' updates the graphics.

2. In arena.py we define classes for the arena properties: the room, obstacle types, powerstrip and floortiles 
3. robot.py defines the Pi-swarm robot functions. High level functions such as 'drive' are composed of the low level functions such as 'driveRightWheelForward'. If you need to add new high level behaviours for the robot then they can be added here, they should make use of the low level functions.
4. proxSensor.py implements the IR sensors that are positioned around the robots. It identifies when a sensor is in contact with another object and the distance that the object is. Note: Please use caution with the Calcdistance function, it may sometimes return an unexpected response. 
5. pygame_framework.py, framework.py and settings.py are from pybox2D and should not need changing. 

Running the simulator

1. cd to the main PiSwarmSimulator directory
2. run simulator using "python Pi-Swarm-Sim.py''

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
