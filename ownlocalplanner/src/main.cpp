#include "plannerLib/plannerlib.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ownLocalPlanner");

    ros::NodeHandle nh;
    plannerLib pl(nh);

    pl.refreshGlobalPosition(nh);

    /*
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    ROS_INFO("END PROGRAM");

    return 0;
}


