#ifndef PARKING_LIB_H
#define PARKING_LIB_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>


class parking_lib
{
public:
    parking_lib();

    ros::NodeHandle ParkingNodeHandle;
    //ros::Publisher joy_servo_publisher;
    //ros::Publisher joy_communication_publisher;
    ros::Subscriber parking_subscriber;

    // FLAGS
    //std_msgs::Int16 emergencyBrake;
    //std_msgs::Int16 controlMode;



    //geometry_msgs::Vector3 servo;
    //std_msgs::Int16MultiArray joy_state;

private:
    void parkingStateCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    std::vector<float> laser_back;
};

#endif // PARKING SLIB_H
