#include <tf/transform_listener.h>
#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "longitudinal_lateral_control");

    ros::NodeHandle node;

    tf::TransformListener listener;
    ROS_INFO(" starting  ");

    ros::Rate rate(2.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO_STREAM("x:   " << transform.getOrigin().x()  << "y:   " << transform.getOrigin().y());

        rate.sleep();
    }

    return 0;

}

