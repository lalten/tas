/// \author Quirin Körner: ga34diz
/// h-file for findpark, see finpark.cpp for explanation

#pragma once

#include <ros/ros.h>

//Open CV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
*/

//std include
#include <iostream>
#include <string>

//ROS
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



//++++ Functions +++++/

int main(int argc, char** argv);
void load_map();
void display_mapReal();
void display_mapTest();
void display_mapMatchParking();

bool calculate_start(int start_x, int start_y, double orientation);
void find_spot(cv::Mat map, cv::Mat templateImg);
void rotate_template(cv::Mat map);
void sendStartPos (double mapStart_x, double mapStart_y, double orientation);
void cornerDetection(cv::Mat map);

//+++++ Variable ++++++/
cv::Mat mapReal;    //Mat matrix for where real map is loaded
cv::Mat mapTest;    //Mat matrix where test map is loaded
std::vector<std::tuple<double,double> > startPositions; //List of X and Y coordinates of Positions to start parking
std::vector<std::vector<int>> possiblePositions;    //this vector contains the best match for each search (position and score)
int maxIndex;  //index of vector position of the best match
int MaxValIndex; //value of the best match
cv::Mat mapParkingLeft;  //template for matching algorithm
std::string tasMapPath; //path to maps
cv::Point startPositionInMap; //goal point where the car should stop and start the parking procedure
move_base_msgs::MoveBaseGoal goal; //goal on map








