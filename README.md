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
 
## Contributions
Laurenz: [Odometry](#odometry)  
Konrad: [Trajectory Rollout](#trajectory)  
Frederik: [SBPL / LQR Controller](#)  
Quirin: [Parking](#parking), [Rotated Template Matching](#)

### Odometry
The enhanced odometry stack uses the following external nodes:
 * [robot_localization __ekf_localization_node__] (http://wiki.ros.org/robot_localization#ekf_localization_node) as extended kalman filter
 * [__laserscan_multi_merger__](https://github.com/iralabdisco/ira_laser_tools) for merging front and back laser scans
 * [__laser_scan_matcher__](http://wiki.ros.org/laser_scan_matcher) Laser-based odometry estimation
 * Custom [__px4flow_node__ fork](https://github.com/lalten/px-ros-pkg) for communication with visual odometry sensor
 * [mtnode.py __xsens_driver__](http://wiki.ros.org/xsens_driver) for IMU communication
 * [__rosserial_arduino__](http://wiki.ros.org/rosserial_arduino) and [__rosserial_python__](http://wiki.ros.org/rosserial_python) for communication during motor encoder processing

The [__tas_odometry__ package](/tas_odometry/package.xml) contains the following nodes created by us:
 * [__perfect_odometry__](/tas_odometry/src/imu_bias_compensation.cpp), which analyzes gazebo link states to provide "perfect" odometry during simulation
 * [__imu_bias_compensation__](/tas_odometry/src/imu_bias_compensation.cpp), which tries to compensate constant acceleration offsets in the IMU (when EKFs gravity compensation is not used)
 * [__optflow_odometry__](/tas_odometry/src/optflow_odometry.cpp), which converts data from the PX4flow sensor to usable odometry (twist) messages. It calculates EKF uncertainty covariance matrix value from image quality data.
 * [__motor_odometry__](/tas_odometry/src/motor_odometry.cpp), which generates odometry (twist messages) from encoder inter-tick times as reported from the encoder microcontroller unit

Code specific to motor encoder processing:
 * [__MotorOdometry__](/Arduino/MotorOdometry/MotorOdometry.ino): Code running on Atmega32u4 MCU. Communicates with ROS via native USB using the rosserial protocol.
 * [__MotorTest__](/Arduino/MotorTest/MotorTest.ino): Test sketch for the MCU. Prints detected encoder revolutions via raw serial.

### Trajectory
#### Preparation and Simulation:

For prepairing the algorithm and for visualization the Bézier Curve was programmed in Matlab. To execute the matlab functions:
 * addpath( [matlab_code](/Matlab Code Konrad/) )
 * read in Costmap [>>Example](/Matlab Code Konrad/Readme.md)
 * alternatePath(Costmap, hight, width, resolution, minRadius)

#### ROS C++ Code:
The [ownLocalPlanner](/ownlocalplanner/) package subscribes the following Nodes:
 * [__Global Plan__](http://www.google.de/)

```
rosrun ownlocalplanner ownlocalplanner
```
 


### Parking:
There are two nodes for the parking process. 
 * The "findpark"-node detects parking spots with template matching or feature extraction. 
 * The "parking"-node runs the actual parking procedure.

#### Find parking spot
Before starting the parking slot detection, you have to accquire a map, includeing at least one parking spot. After that run:
```
roslaunch findpark findpark.launch
```
Once the car reached the start position run the "parking"-node

Note: For showcasing without car, the findpark.cpp is now in testmode. This means, that the testMap.pgm is loaded instead of the real map. If you want, you may change this in the code with the TEST-flag. 

For more detailed information how the findpark-node works, look into the readme: [__/findpark/readme__](/findpark/readme)

#### Park the car
Once the car is at the right starting position (calculated by "findpark" or the marked spots at the floor) start:
```
rosrun parking parking
```
The parking node uses the raw data of the messages
* /scan_back
* /scan

These messages contain the laser distance table. 
The movements a directly published to the node
* /servo

For more information how the parking-node works, look into the readme: [__/parking/readme__](/parking/readme)
