tas_car
=======
Basic setup for the TAS stack

This repository has two branches: master and simulation.	
	1. Branch "master": for working on the real car
	2. Branch "simulation": for working on gazebo simulation

=================================================================

1. Branch "master":
	- Please follow the instruction on Moodle for better understanding about the package
	- This branch is only used for real car. It will not work on your own machine

=================================================================
2. Branch "simulation":
	- Provide a gazebo simulation model based on ackermann_vehicle package on ROS
	- Please follow the instructions in INSTALL.md to install all necessary packages
	- For launching simulator run:
		`roslaunch tas_simulator startSimulation.launch`
	- To start navigation stack in simulation run
		`roslaunch tas_simulator startNavigation.launch`	
	- To change to autonomous mode use
		`rostopic pub /wii_communication std_msgs/Int16MultiArray`
	hit "Tab" a few times and change that 0 to 1
	- And publish goals:
		`rosrun simple_navigation_goals simple_navigation_goals_node` 
   REMEMBER: This branch only provides you a gazebo simulation model, additional details and control strategies need to be added to fullfil your tasks
			

