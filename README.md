tas_car
=======
Basic setup for the TAS stack

This repository has two branches: master and simulation.	
	1. Branch "master": for working on the real car
	2. Branch "simulation": for working on gazebo simulation

# Branch "master":
 - Please follow the instruction on Moodle for better understanding about the package
 - This branch is only used for real car. It will not work on your own machine

## ROS via tcp/ip:
More info on the ROS network setup is on [ROS Wiki](http://wiki.ros.org/ROS/NetworkSetup).

 * ssh into the car: `ssh tas_group_06@10.42.0.1` (or create an ssh [config](http://nerderati.com/2011/03/17/simplify-your-life-with-an-ssh-config-file/) file so you can just do `ssh vettel` and skip password prompt)
 * set the **ROS_IP** to the car's IP address: `export ROS_IP=10.42.0.1`
 * start a roscore in background and disown it (so you'll be able to close the terminal) - `roscore &`, `disown`
 * launch your files in background and disown again: `roslaunch tas run_system.launch &`, `disown`
 * on your laptop, start a terminal and connect to the car via ethernet or wifi
 * set the ROS master location: `export ROS_MASTER_URI=http://10.42.0.1:11311`
 * tell ros your ip: `export ROS_IP=10.42.0.100` -- **adjust this to your ip**, which you can see using `ifconfig`
 * start your local nodes: `rosrun rviz rviz`, or `rosrun image_view image_view image:=/px4flow/camera_image`

# Branch "simulation":
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
			

