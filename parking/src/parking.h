#pragma once

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Vector3.h>


void parkingCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
int main(int argc, char** argv);
bool checkBox (float laser_dist);
int getNumBoxes (float laser_dist);
void stopEngine();
void setSpeed(int speedOffset, int turnOffset);
ros::Publisher parking_servo_publisher;






