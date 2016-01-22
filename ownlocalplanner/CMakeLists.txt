cmake_minimum_required(VERSION 2.8.3)
project(ownlocalplanner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  nav_msgs
  sensor_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(ownlocalplanner src/main.cpp src/plannerLib/plannerlib.cpp)



## Specify libraries to link a library or executable target against
target_link_libraries(ownlocalplanner ${catkin_LIBRARIES})


