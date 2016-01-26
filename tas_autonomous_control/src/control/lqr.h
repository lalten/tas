#ifndef LQR_H
#define LQR_H

using namespace std;

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>

#include <vector>

#define PI 3.14159265


class iir
{
    public:
    iir()
    {
        inited = 0;
    }
    double filter(double in, double alpha)
    {
        if(inited ==0)
            out_last = in;
        inited = 1;
        double out = out_last*(1-alpha) + alpha* in;
        out_last = out;
        return out;
    }
    private:
        int inited;
        double out_last;
};

class lqr
{
    public:
        lqr();
        ros::Subscriber glpath_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber odom_sub_;
        ros::NodeHandle node;
        double control();                   // calculate control output and publish
        void getclosestpoint();             // calculate piont on the path with smallest distance to vehicle
        double mapcoord[3];                 //x, y, eulerangle z in map frame in **degress**!
        double last_mapcoord[3];            //x, y, eulerangle z in map frame
        void getPath();                     //read in the global path
        double get_z_euler_from_quad(vector <double> q);
        vector <double> get_quad_from_euler(double alpha);
        vector <double> closestpt;          //initialize with zeros
        void visualize();                   //plot some markers
        int inited;                         //value indicating that a global path has been received
        vector < vector <double> > glpath;  //vector of n points containing (x,y, z-orientation) vectors
        void estimate_state();              //estimate the current velocity and angular veloctiy
        ros::Time timenow;                  // time stamps used in state estimate
        ros::Time timelast;
        double imu_angular_z_vel_uf;
        double angular_vel_offset;
        void test_motor();

        double des_vel;         //desired velocity derived from path geometry in m/s
        int des_dir;            //1 forward -1 backwards

    private:
        double Kvec[3];                     // LQR-control gains (computed by Matlab)
        double Kvec_1[3];                     // LQR-control gains at vel_ref (computed by Matlab)
        double cmd_thrust;
        double vel_ref;                     //velocity reference value for time-varying control interpolation
        double err[3];                      // errors of the three states
        vector < double > des_speed_vec;    //vector containing desired velocity for each point on the path
        vector < double > distance_to_last; //distance from one point to the other on the path            
        vector < double > path_curv;        //the curvature of the path
        vector < double > angle_diff_per_m;        //the curvature of the path
        vector < int > dir_vec;          // 1 forward -1 backward
        void calc_des_speed();
        void glpathCallback(const nav_msgs::Path::ConstPtr& path);      //called when there is a new global path
        void imuCallback(const sensor_msgs::Imu::ConstPtr& data);      //called when there is a new global path
        void odomCallback(const nav_msgs::Odometry::ConstPtr& data);      //called when there is a new global path
        void publish_sim();
        void publish_car();

        void test_speed_control();

        double odom_vel;
        double max_vel;          //maximum allowd velocity
        double corner_speed;
        double acc_distance;     //distance from pathstart after which max_vel shall be reached, in m
        double decc_distance;    //distance to pathend in which max_vel shall to 0, in m
        double dphi;            //current estimate of angular velocity in deg/s
        double vel;             //current estimate of linear velocity in m/s

        double steering_deg;    //positive makes left turn
        double int_err;         //integrated error of PI-speed-controller

        iir filter_phi, filter_vel, filter_imu;
        ros::Publisher pub_ball;
        ros::Publisher pub_arrow;
        ros::Publisher pub_arrow_array;
        ros::Publisher pub_vel;
        ros::Publisher pub_cmd_vel;
        ros::Publisher pub_ackermann_sim;
        ros::Publisher pub_servo;
};

#endif // LQR_H
