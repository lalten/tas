#include "parking_lib/parking_lib.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_node");
    ros::NodeHandle nh;
    parking_lib pl(&nh);

    ros::Rate loop_rate(50);

    ROS_INFO("Started Parking Node");

    while(ros::ok())
    {
        ROS_INFO("Parking Test, spin once");
        //teleop_joy.joy_communication_publisher.publish(teleop_joy.joy_state);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
