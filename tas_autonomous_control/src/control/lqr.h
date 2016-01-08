#ifndef LQR_H
#define LQR_H

#include "ros/ros.h"
using namespace std;

#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>

#define PI 3.14159265

class lqr
{
    public:
        lqr(ros::NodeHandle node);
        ros::Subscriber glpath_sub_;
        //ros::NodeHandle node;
        double getsteering();
        //void calc_error();
        double currentpose[3];
        void getclosestpoint();
        double mapcoord[3]; //x, y, eulerangle z in map frame
        void getPath();
        double get_z_euler_from_quad(vector <double> q);
        vector <double> closestpt; //initialize with zeros
        void visualize();

    private:
        double Kvec[3];
        //double visualize_closest_point();
        double ctrl_error[3];
        vector < vector <double> > glpath; //vector of n points containing (x,y, z-orientation) vectors

        void glpathCallback(const nav_msgs::Path::ConstPtr& path);
};

#endif // LQR_H
