##
## Quirin Körner // ga34diz
##

cmake_minimum_required(VERSION 2.8.3)
project(findpark)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge 
  image_geometry
)

find_package(OpenCV REQUIRED)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(findpark src/findpark.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(findpark ${catkin_LIBRARIES})




