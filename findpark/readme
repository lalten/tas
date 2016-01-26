##################### 
Readme for running the findpark node.
#####################
author: Quirin Körner | ga34diz 
#####################
cmake is changed, do not forget:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

! You will need open CV (2.4.x) to run this.
#####################

If you want to run it with the real car. Roslaunch the run_system
#roslaunch tas run_system

Once you want to find a parking spot start findpark. You need to generate a map before (make a real round).
#roslaunch findpark findpark.launch

Once the start position for parking is reached. Run the parking node.
#rosrun parking parking

######################
How it works:
The findparking needs a created map, with at leat one reachable parking spot.
This map must be stored in the standard map folder.

Once the findpark node is started it will search for possible parking spots.
Following limitations are set at the moment:
- A parking spot has to be at a wall (paralell)
- The parkingspot is limited by two paper boxes
- The distance between the paper boxes is exactly like thee distance of the "original" parking spot
These restrictions can be modified in the code.

Once the car detected a possible parking spot. The current goal of the pathplanning is changed to 50cm distance to wall (Y) and -25cm distance (X) to the first box. If the goal position is not reachable (wall, out of map) it's excluded of the possible parking spots list.

As the parking node only support parkingspots left of the car, the findpark node only uses parkingspots at the left.

Once the goal is reached the findpark node is shut down, and the user can start the parking node manually. This starts the parking procedure discribed in the readme of the parking node.
#######################
HOW IT WORKS
######################
Harris corner detector

The first approach was to use corner detection to find a parking spot. As the results weren't robust enough, I switched to the roatated template detection. A reason for the poor robustness is the the low resultion of the map. With higher resultion the results are better, but this causes problems with the localization during the race. Therefore I tried template matching.

######################
Rotated templated matching

Template matching uses a template (map of parking spot) which shows the ideal parking spot (see /tas/config/Map_server/static). which is shifted horizontal and vertical over the recorded/test map of the whole corridor.
But as the yaw of the map is not known (build in first round) the template matching does not give good results. Therefore I implemented a turning algorithm of the template map. The matching is done with 1° offset each round. The highest score of each round is saved and compared to all rounds. The highest score is the location of the parking spot with it's corresponding angle.

The matching algorithm give back the left-top corner of the best match, therefore the location of the position to start the parking procedure has to be calculated (sin, cos, with angle of matching). The pixel position in the map, has to be converted in cordinates to set the goal of move_base.

The car drives to the goal, if it's in autonomes mode. Once reached the parking-node has to be launched manualy.

Note: To save computing time, the rotation is currently limmited to 0-30° (can be changed in findpark.cpp). Therefore only parking spots in the corridor at the frontdoors can be found.
