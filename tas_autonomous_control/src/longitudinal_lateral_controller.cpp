#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>
#define PI 3.14159265

#include "control/lqr.h"

using namespace std;

double get_z_euler_from_quad(vector <double> q)
{
    return atan2(   2*(q.at(0)*q.at(3) + q.at(1)*q.at(2)), 1- 2*(pow(q.at(2),2) + pow(q.at(3),2)));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "longitudinal_lateral_control");
    ros::NodeHandle node;

    tf::TransformListener listener;

    ros::Rate rate(1);  //in Hz

    lqr lqr1(node);

    ROS_INFO_STREAM("entering loop");
    while (node.ok()){

        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1).sleep();
            continue;
        }
        ROS_INFO_STREAM("x:   " << transform.getOrigin().x()  << "y:   " << transform.getOrigin().y());
        ROS_INFO_STREAM("angle of rotation: " << transform.getRotation().getAngle()/PI*180.0);
        ROS_INFO_STREAM("axis of rotation: " << transform.getRotation().getAxis().getX() << "  " << transform.getRotation().getAxis().getX() << "  " << transform.getRotation().getAxis().getZ() );

        lqr1.mapcoord[0] = transform.getOrigin().x();
        lqr1.mapcoord[1] = transform.getOrigin().x();
        lqr1.mapcoord[2] = transform.getRotation().getAngle()/PI*180.0* transform.getRotation().getAxis().getZ();  //this assumes rotation only around z axis!

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}

