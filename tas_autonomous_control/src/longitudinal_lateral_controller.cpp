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

        if(lqr1.inited==0) //if not inited yet:
        {
            memcpy(lqr1.last_mapcoord, lqr1.mapcoord, 3*sizeof(double));
            lqr1.timelast = ros::Time::now();
            ROS_INFO_STREAM("inited" << lqr1.inited);

            while(lqr1.angular_vel_offset == 0.0)     // spin as long as /imu message has not been read
            {
                ros::spinOnce();
                lqr1.angular_vel_offset = lqr1.imu_angular_z_vel_uf;
            }
        }

        //ROS_INFO_STREAM("Current Pos   x:   " << lqr1.mapcoord[0]  << "y:   " << lqr1.mapcoord[0] <<   "z-angle:   " << lqr1.mapcoord[2] );

        /*lqr1.des_vel =0;
        while(1)
        {
            for(int i = 0; i<5 ; i+= 0.03)
            {
                lqr1.des_vel = i;
                test_speed_control();
            }

            for(int i = 5; i>-5; i-= 0.03)
            {
                lqr1.des_vel = i;
                test_speed_control();
            }
        }*/


        if( lqr1.inited == 1)
        {           
            lqr1.getclosestpoint();
            ROS_INFO_STREAM("got closest pt");
            lqr1.visualize();
            ROS_INFO_STREAM("visualize");
            lqr1.estimate_state();
            ROS_INFO_STREAM("state est");
            lqr1.control();
            ROS_INFO_STREAM("closest point on path:   " << lqr1.closestpt.at(0) << "  " << lqr1.closestpt.at(1) << "  " << lqr1.closestpt.at(2));
            ROS_INFO_STREAM("glpath size:   " << lqr1.glpath.size());
            /*for (int i=0; i< lqr1.glpath.size()/1.0; i++)
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

