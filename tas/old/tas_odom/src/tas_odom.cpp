/**
 * This node forwards the hector pose of the robot. It generates the estimated
 * hector slam position and the velocities  of the robot (based on the kinematic
 * equations for an Ackermann-steered vehicle) and publishes tf messages to
 * topic /tf and odom messages to topic /odom.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
//#include <razor_imu_9dof/RazorImu.h>

// define macros
#define WHEELBASE 0.36 // distance between the rear axel and front one in m
#define CONVERSION_FACTOR 1./3.6 // factor to convert velocities from km/h to m/s

// declare and initialize global variables
double x_ = 0.0; // x position in m
double y_ = 0.0; // y position in m
double th_ = 0.0;
geometry_msgs::Quaternion hector_quaternion;
double v_ = 0.0; // (linear) velocity in m/s
double sa_ = 0.0; // steering angle in rad
double pre_x = 0.0; // previous x position in m
double pre_y = 0.0; // previous y position in m
double pre_th = 0.0; // previous orientation

/**
 * Callback function to receive the estimated position from the hector node.
 */
void hectorPoseCallback(const geometry_msgs::PoseStampedConstPtr& hector_msg) {
    x_ = hector_msg->pose.position.x; // hector x position
    y_ = hector_msg->pose.position.y; // hector y position

    hector_quaternion.z = hector_msg->pose.orientation.z; // hector z orientation
    hector_quaternion.w = hector_msg->pose.orientation.w; // hector z orientation
}


/**
 * Main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "hector_odometry"); // init and set name
    ROS_INFO("hector_odometry node started!");
    ros::NodeHandle nh;

    // set up subscriber and publisher
    ros::Subscriber hector_pose_subscriber = nh.subscribe("slam_out_pose", 1, hectorPoseCallback); // buffer only 1 slam_out_pose message
    ros::Publisher wheel_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50); // buffer last 50 odom frames
    //    ros::Publisher visual_odom_pub = nh.advertise<nav_msgs::Odometry>("visual_odom", 50); // buffer last 50 odom frames

    tf::TransformBroadcaster odom_broadcaster;

    // declare and initialize messages
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // declare and initialize messages
    nav_msgs::Odometry wheel_odom;
    wheel_odom.header.frame_id = "odom";
    wheel_odom.child_frame_id = "base_link";


    // declare and initialize state space variables
    double x_dot = 0.0;
    double y_dot = 0.0;
    double th_dot = 0.0;


    // initialize node parameters from launch file
    int rate; // frequency in Hz
    ros::NodeHandle private_nh("~");
    private_nh.getParam("rate", rate); // frequency

    ros::Rate r(rate); // set up ROS loop rate

    // initialize time variables
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(nh.ok())
    {
        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();

        th_ = tf::getYaw(hector_quaternion);

        //ROS_INFO("hector yaw = %f", th_);

        x_dot = (x_ - pre_x)/dt;
        y_dot = (y_ - pre_y)/dt;
        th_dot = (th_ - pre_th)/dt;

        // publish the transform over tf
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
        odom_broadcaster.sendTransform(odom_trans);	// send transform

        // publish the wheel odometry message
        wheel_odom.header.stamp = current_time;
        wheel_odom.pose.pose.position.x = x_;
        wheel_odom.pose.pose.position.y = y_;
        wheel_odom.pose.pose.position.z = 0.0;
        wheel_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
        wheel_odom.twist.twist.linear.x = x_dot;
        wheel_odom.twist.twist.linear.y = y_dot;
        wheel_odom.twist.twist.angular.z = th_dot;
        wheel_odom_pub.publish(wheel_odom);	//publish message

        // update time and sleep with loop rate
        last_time = current_time;
        pre_x = x_;
        pre_y = y_;
        pre_th = th_;

        r.sleep();
    }
}

