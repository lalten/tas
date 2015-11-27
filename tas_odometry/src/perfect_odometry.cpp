#include <tas_odometry/perfect_odometry.h>

ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster;

void linkStateCallBack(const gazebo_msgs::LinkStates::ConstPtr& link_states) {
	int base_link_index = std::distance(link_states->name.begin(),
			std::find(link_states->name.begin(), link_states->name.end(),
					"ackermann_vehicle::base_link"));

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x =
			link_states->pose[base_link_index].position.x;
	odom_trans.transform.translation.y =
			link_states->pose[base_link_index].position.y;
	odom_trans.transform.translation.z =
			link_states->pose[base_link_index].position.z;
	odom_trans.transform.rotation =
			link_states->pose[base_link_index].orientation;

	//send the transform
	odom_broadcaster->sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose = link_states->pose[base_link_index];

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist = link_states->twist[base_link_index];

	//publish the message
	odom_pub.publish(odom);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Subscriber link_state_sub = n.subscribe(
			"/ackermann_vehicle/gazebo/link_states", 1000, linkStateCallBack);

	odom_broadcaster = new tf::TransformBroadcaster();

	ros::spin();
}
