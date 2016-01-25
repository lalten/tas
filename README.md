# TAS WS15/16 - Group 6
TUM LSR Technik Autonomer Systeme WS15/16  
Group 6 (Laurenz Altenmüller, Frederik Ebert, Quirin Körner, Konrad Vowinckel)  
https://www.lsr.ei.tum.de/en/courses/vorlesungen/wintersemester/master/technik-autonomer-systeme/

## Installation
See installation [instructions](INSTALL.md).

## Running the software

### Simulation
```
roslaunch tas_simulator startSimulation.launch
roslaunch tas_simulator startNavigation.launch
```

### Hardware
```
roslaunch tas hardware.launch
roslaunch tas odom.launch
roslaunch tas move_base.launch
```

## ROS via tcp/ip:
More info on the ROS network setup is on [ROS Wiki](http://wiki.ros.org/ROS/NetworkSetup).

__Prerequisites__: Append the ROS_HOSTNAME line to __both__ the car's and your client machine's .bashrc:  
```
export ROS_HOSTNAME=$(hostname).local
```

 * if you are using ROS in a virtual machine, make sure it is set to bridged network mode.
 * ssh into the car: `ssh tas_group_06@10.42.0.1` (or create an ssh [config](http://nerderati.com/2011/03/17/simplify-your-life-with-an-ssh-config-file/) file so you can just do `ssh vettel` and skip password prompt) to launch your files
 * on your laptop, start a terminal and connect to the car via ethernet or wifi
 * set the ROS master location: `export ROS_MASTER_URI=http://vettel.local:11311`
 * start your local nodes: e.g. `rviz`, or `rosrun image_view image_view image:=/px4flow/camera_image`
 
 ## Parking:
 There are two nodes for the parking process. 
 *The "findpark"-node detects parking spots with template matching or feature extraction. 
 *The "parking"-node runs the actual parking procedure.
 
 ### Find parking spot
 Before starting the parking slot detection, you have to accquire a map, includeing at least one parking spot. After that run:
 '''
 roslaunch findpark findpark.launch
 '''
 Once the car reached the start position run the "parking"-node
 
 Note: For showcasing without car, the findpark.cpp is now in testmode. This means, that the testMap.pgm is loaded instead of the real map. If you want, you may change this in the code with the TEST-flag. 
 
 For more information about the findpark-node look into the readme at 
 '''
 tas/src/findpark/
 '''
 
 ### park the car
 Once the car is at the right starting position (calculated by "findpark" or the marked spots at the floor) start:
 '''
 rosrun parking parking
 '''
 
 
 
 


