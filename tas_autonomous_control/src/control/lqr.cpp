#include "lqr.h"

lqr::lqr(ros::NodeHandle node)
{    
      Kvec[0]=0.5232;
      Kvec[1]=-3.0107;
      Kvec[2]=-5.4772;

      ROS_INFO_STREAM("constructor");

      glpath_sub_ = node.subscribe<nav_msgs::Path>("/move_base_node/TrajectoryPlannerROS/global_plan", 100, &lqr::glpathCallback,this);
}

double lqr::getsteering()
{
    cout << "test" << endl;
}

void lqr::getclosestpoint()
{
    double shortestdistance = 100000;
    int indclose = 0;
    for(int i = 0; i<glpath.size(); i++)
    {
        double px = glpath.at(i).at(0);
        double py = glpath.at(i).at(1);

        double distance =  sqrt(pow(px-mapcoord[0],2) + pow(py-mapcoord[1],2));
        if(distance < shortestdistance)
            shortestdistance = distance;
            indclose = i;
    }
    closestpt.push_back( glpath.at(indclose).at(0));
    closestpt.push_back( glpath.at(indclose).at(1));
    closestpt.push_back( glpath.at(indclose).at(2));
}


double lqr::get_z_euler_from_quad(vector <double> q) //uses quaternion definiton : w, x, y, z !!
{
    return atan2( 2*(q.at(0)*q.at(3) + q.at(1)*q.at(2)), 1- 2*(pow(q.at(2),2) + pow(q.at(3),2)));
}

void lqr::visualize()
{
    ros::NodeHandle node;
    ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "closest_pt", 0 );

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "lqr";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = mapcoord[0];
    marker.pose.orientation.y = mapcoord[1];
    marker.pose.orientation.z = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );
}

void lqr::glpathCallback(const nav_msgs::Path::ConstPtr& path)
{
    ROS_INFO_STREAM("callback");


    int num_points = path->poses.size();
    /*
    ROS_INFO_STREAM("path->header.frame_id:" << path->header.frame_id );
    ROS_INFO_STREAM("number of poitns:" << num_points );
    double x = path->poses.at(0).pose.position.x;
    double y = path->poses.at(0).pose.position.y;
    ROS_INFO_STREAM("poses.at(0).pose.position x:" << x << "  y:  " << y );

    //getting the orientation:
    vector <double> q(4);  //quaternion
    q.at(0) = path->poses.at(num_points-1).pose.orientation.w;
    q.at(1) = path->poses.at(num_points-1).pose.orientation.x;
    q.at(2) = path->poses.at(num_points-1).pose.orientation.y;
    q.at(3) = path->poses.at(num_points-1).pose.orientation.z;


    ROS_INFO_STREAM("orientation at num_points,  x: " << q.at(0) << " y: "<< q.at(1)<< " z: "<< q.at(2)<<  " w: " << q.at(3));
    ROS_INFO_STREAM("euler, z angle:  " << get_z_euler_from_quad(q)/PI*180.0);

    x = path->poses.at(num_points-1).pose.position.x;
    y = path->poses.at(num_points-1).pose.position.y;
    ROS_INFO_STREAM("poses.at(num_points).pose.position x:" << x << "  y:  " << y );
   */

    for(int i = 0; i<num_points; i++)
    {
        //getting the orientation:
        vector <double> q(4);  //quaternion
        q.at(0) = path->poses.at(i).pose.orientation.w;
        q.at(1) = path->poses.at(i).pose.orientation.x;
        q.at(2) = path->poses.at(i).pose.orientation.y;
        q.at(3) = path->poses.at(i).pose.orientation.z;

        double zangle = get_z_euler_from_quad(q)/PI*180.0;

        //getting the path points in x and y:
        double x = path->poses.at(0).pose.position.x;
        double y = path->poses.at(0).pose.position.y;

        if(i==0)
        {
            ROS_INFO_STREAM("path coords at 0: x"  << x << "  y:  " << y );
            ROS_INFO_STREAM("orientation at 0,  w: " << q.at(0) << " x: "<< q.at(1)<< " y: "<< q.at(2)<<  " z: " << q.at(3));
            ROS_INFO_STREAM("euler, z angle:  " << get_z_euler_from_quad(q)/PI*180.0);
        }

        vector <double > pathpoint;
        pathpoint.push_back(x);
        pathpoint.push_back(y);
        pathpoint.push_back(zangle);
        glpath.push_back(pathpoint);
    }

    getclosestpoint();
    visualize();
    ROS_INFO_STREAM("closest point on path:   " << closestpt.at(0) << "  " << closestpt.at(1) << "  " << closestpt.at(2));

}



