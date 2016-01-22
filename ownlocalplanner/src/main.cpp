#include "plannerLib/plannerlib.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ownLocalPlanner");
    plannerLib pl;

    pl.refreshGlobalPosition();

    /*
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    return 0;
}


