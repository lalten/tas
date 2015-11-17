#ifndef JOY_LIB_H
#define JOY_LIB_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <wiimote/State.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

#define SCALE_FAKTOR_STEERING   500

class joy_lib
{
public:
    joy_lib();

    ros::NodeHandle nh;
    ros::Publisher joy_servo_publisher;
    ros::Publisher joy_communication_publisher;
    ros::Subscriber joy_subscriber;

    // FLAGS
    std_msgs::Int16 emergencyBrake;
    std_msgs::Int16 controlMode;

    double SCALE_FACTOR_THROTTLE;

    geometry_msgs::Vector3 servo;
    std_msgs::Int16MultiArray joy_state;

private:
    void joyStateCallback(const sensor_msgs::Joy::ConstPtr& msg);

    void msgInit(std_msgs::Int16MultiArray& msg);
};
