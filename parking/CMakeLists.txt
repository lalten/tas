cmake_minimum_required(VERSION 2.8.3)
project(parking)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(parking src/main.cpp src/parking_lib/parking_lib.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(parking ${catkin_LIBRARIES})


