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

    ros::Rate rate(30);  //in Hz

    lqr lqr1;

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

        lqr1.mapcoord[0] = transform.getOrigin().x();
        lqr1.mapcoord[1] = transform.getOrigin().y();
        lqr1.mapcoord[2] = transform.getRotation().getAngle()/PI*180.0* transform.getRotation().getAxis().getZ();  //this assumes rotation only around z axis!

        if(lqr1.inited==0){
            memcpy(lqr1.last_mapcoord, lqr1.mapcoord, 3*sizeof(double));
            lqr1.timelast = ros::Time::now();
            ROS_INFO_STREAM("inited " << lqr1.inited);
        }

        //ROS_INFO_STREAM("Current Pos   x:   " << lqr1.mapcoord[0]  << "y:   " << lqr1.mapcoord[0] <<   "z-angle:   " << lqr1.mapcoord[2] );

        if( lqr1.inited == 1)
        {           
            lqr1.getclosestpoint();            
            lqr1.visualize();
            lqr1.estimate_state();
            lqr1.control();
            //ROS_INFO_STREAM("closest point on path:   " << lqr1.closestpt.at(0) << "  " << lqr1.closestpt.at(1) << "  " << lqr1.closestpt.at(2));
            /*ROS_INFO_STREAM("glpath size:   " << lqr1.glpath.size());
            for (int i=0; i< lqr1.glpath.size()/1.0; i++)
            {
                int j= i*1;
                ROS_INFO_STREAM("glpath_point x: " << lqr1.glpath.at(j).at(0) << " y: " << lqr1.glpath.at(j).at(1) <<  " zangle: " << lqr1.glpath.at(j).at(2));

            }*/
        }

        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}
