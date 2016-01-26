#pragma once

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Vector3.h>
#//include <sound_play/sound_play.h>


void parkingCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void scanFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void setSpeed(int speedOffset, int turnOffset);
int main(int argc, char** argv);
bool checkBox (float laser_dist);
int getNumBoxes (float laser_dist);
void stopEngine();
void setSpeed(int speedOffset, int turnOffset);
int calcTurnRate (double distance);
ros::Time zeit_start;
ros::Publisher parking_servo_publisher;








