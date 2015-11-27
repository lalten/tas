<<<<<<< HEAD
## Install SDL library

`sudo apt-get install libsdl2-dev libsdl-image1.2-dev`

## Install ROS Packages

`sudo apt-get install ros-indigo-joy ros-indigo-move-base-msgs ros-indigo-wiimote ros-indigo-controller-manager ros-indigo-hector-gazebo ros-indigo-ros-controllers ros-indigo-ros-control ros-indigo-gazebo-ros-control libiw-dev`

`sudo apt-get install ros-indigo-hector-mapping ros-indigo-map-server ros-indigo-amcl ros-indigo-move-base`
=======
# ROS Package dependencies

## General dependencies
```
sudo apt-get install \
ros-indigo-hector-slam \
ros-indigo-map-server \
ros-indigo-amcl
```

## EKF dependencies
```
sudo apt-get install \
ros-indigo-robot-localization
```

## Simulation dependencies 
```
sudo apt-get install \
ros-indigo-move-base ros-indigo-move-base-msgs \
ros-indigo-joy ros-indigo-wiimote \
ros-indigo-controller-manager ros-indigo-ros-controllers ros-indigo-ros-control \
ros-indigo-cob-gazebo-ros-control ros-indigo-hector-gazebo
```
>>>>>>> ekf

