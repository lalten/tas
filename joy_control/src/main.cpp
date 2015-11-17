#include "joy_lib/joy_lib.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_joy");
    joy_lib teleop_joy;

    static int count = 0;

    ROS_INFO("Started Joy Node");

    while(ros::ok())
    {
        teleop_joy.joy_communication_pub.publish(teleop_joy.joy_state);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
