/* Perfect Odometry
 *
 * This node reads in gazebo link states and publishes the corresponding odom-
 * base transform as if it came from (a very good) odometry.
 *
 * ROS parameters:
 * odom_frame - frame_id of odom tf
 * base_frame - frame_id of base tf
 * pub_odom_base_tf - determines if transforms should be published
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

#include <tas_odometry/perfect_odometry.h>

ros::Publisher odom_publisher;
tf::TransformBroadcaster* odom_broadcaster;
std::string odom_frame;
std::string base_frame;
bool pub_tf;

void linkStateCallBack(const gazebo_msgs::LinkStates::ConstPtr& link_states) {
	int base_link_index = std::distance(link_states->name.begin(),
			std::find(link_states->name.begin(), link_states->name.end(),
					"ackermann_vehicle::base_link"));

  if(pub_tf) {
	  //first, we'll publish the transform over tf
	  geometry_msgs::TransformStamped odom_trans;
	  odom_trans.header.stamp = ros::Time::now();
	  odom_trans.header.frame_id = odom_frame;
	  odom_trans.child_frame_id = base_frame;

	  odom_trans.transform.translation.x = link_states->pose[base_link_index].position.x;
	  odom_trans.transform.translation.y = link_states->pose[base_link_index].position.y;
	  odom_trans.transform.translation.z = link_states->pose[base_link_index].position.z;
	  odom_trans.transform.rotation = link_states->pose[base_link_index].orientation;

	  //send the transform
	  odom_broadcaster->sendTransform(odom_trans);
  }

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = odom_frame;

	//set the position
	odom.pose.pose = link_states->pose[base_link_index];

	//set the velocity
	odom.child_frame_id = base_frame;
	odom.twist.twist = link_states->twist[base_link_index];

	//publish the message
	odom_publisher.publish(odom);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle n;

  n.param<std::string>("/perfect_odometry/odom_frame", odom_frame, "odom");
  n.param<std::string>("/perfect_odometry/base_frame", base_frame, "base_link");
  n.param<bool>("/perfect_odometry/pub_odom_base_tf", pub_tf, true);

  //ROS_INFO("Perfect odometry: using %s as odom frame", odom_frame.c_str());

	odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);

	ros::Subscriber link_state_sub = n.subscribe(
			"/ackermann_vehicle/gazebo/link_states", 1000, linkStateCallBack);

	odom_broadcaster = new tf::TransformBroadcaster();

	ros::spin();
}
