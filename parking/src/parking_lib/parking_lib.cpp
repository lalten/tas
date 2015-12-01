#include "parking_lib.h"

parking_lib::parking_lib() {
    //joy_servo_publisher = nh.advertise<geometry_msgs::Vector3>("servo",1);
    //joy_communication_publisher = nh.advertise<std_msgs::Int16MultiArray>("wii_communication",1);
    parking_subscriber = ParkingNodeHandle.subscribe("/LaserScan", 100,&parking_lib::parkingStateCallback, this);

    //msgInit(joy_state);


}

void parking_lib::parkingStateCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    laser_back.clear();
    laser_back = msg->ranges;
        //joy_servo_publisher.publish(servo);
    ROS_INFO("Laser Daten bekommen");

}

