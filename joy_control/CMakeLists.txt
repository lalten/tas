cmake_minimum_required(VERSION 2.8.3)
project(joy_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  wiimote
  sensor_msgs
  turtlesim
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(joy_node src/main.cpp src/wii_lib/wii_lib.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(wii_node ${catkin_LIBRARIES})


