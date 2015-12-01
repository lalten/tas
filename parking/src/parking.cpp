#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#define SUM_DIST 10

void parkingCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
   {
    float temp = 0;
    for (int i=0; i<SUM_DIST; i++)
    {
        temp = temp + msg->ranges[1];
    }
    temp = temp / SUM_DIST;
 ROS_INFO("I heard: %lf",temp);
 }

int main(int argc, char** argv) {

    ros::init(argc, argv, "parking_listener");

	ros::NodeHandle n;

    ros::Subscriber laser_back_sub = n.subscribe("/scan_back", 1000, parkingCallback);

	ros::spin();
}
