# ROS Package dependencies

## General dependencies
```
sudo apt-get install \
ros-indigo-hector-slam \
ros-indigo-map-server \
ros-indigo-amcl \
ros-indigo-robot-localization \
ros-indigo-sound-play \
ros-indigo-tf2-bullet \
libgstreamer-0.10-dev libgstreamer-plugins-base0.10-dev
```

```
roscd && cd ../src
git clone https://github.com/lalten/px-ros-pkg.git
cd ..
catkin_make
```

## Simulation dependencies 
```
sudo apt-get install \
ros-indigo-move-base ros-indigo-move-base-msgs \
ros-indigo-joy ros-indigo-wiimote \
ros-indigo-controller-manager ros-indigo-ros-controllers ros-indigo-ros-control \
ros-indigo-cob-gazebo-ros-control ros-indigo-hector-gazebo
```

## Arduino odometry development
```
# Install packages
sudo apt-get install \
arduino \
ros-indigo-rosserial ros-indigo-rosserial-arduino
```
Note that a much newer version of the [Arduino IDE](http://arduino.cc/en/Main/Software) is available from its website.

Next, create the `ros_lib` as instructed in the [rosserial_arduino Turoial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Install_ros_lib_into_the_Arduino_Environment). You may also copy the directory from [tas/Arduino/ros_lib](Arduino/ros_lib).

For development, I used the [Eclipse Arduino Plugin](http://eclipse.baeyens.it/), hence the .project files.

## Possibly also necessary?

```
sudo apt-get install \
libsdl2-dev libsdl-image1.2-dev
```

