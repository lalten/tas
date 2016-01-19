/* Motor Odometry
 *
 * This node reads in encoder data from an Arduino monitoring a sensored
 * brushless motor. It append the length driven (according to the motor
 * encoders) to the X dimension of this pose.
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>

ros::Publisher pose_publisher;
ros::Publisher encoder_publisher;

geometry_msgs::PoseWithCovarianceStamped pose;
uint32_t seq = 0;
int32_t encoder_abs = 0;

std::string frame_id;
double ticks_per_meter;
double uncertainty_fixed;

// We get new encoder values here
void encoder_callback(const std_msgs::Int32::ConstPtr& encoder_change) {

	// Publish integrated/absolute encoder value
	encoder_abs += encoder_change->data;
	std_msgs::Int32 msg;
	msg.data = encoder_abs;
	encoder_publisher.publish(msg);

	// Convert length to meter
	double change_meter = (double) encoder_change->data / ticks_per_meter;

	// Add it to position
	pose.pose.pose.position.x += change_meter;

	// Update header
	pose.header.frame_id = frame_id;
	pose.header.stamp = ros::Time::now();
	pose.header.seq = seq++;

	// Send out message
	pose_publisher.publish(pose);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_odometry");
	ros::NodeHandle n;

	// ROS params
	n.param<double>("ticks_per_meter", ticks_per_meter, 100);
	n.param<std::string>("frame_id", frame_id, "base_motor");
	n.param<double>("uncertainty_fixed", uncertainty_fixed, 1e-3);

	// Set initial orientation to zero (aligned)
	geometry_msgs::Quaternion orientation_msg;
	orientation_msg.w = 1.0; // = no rotation
	pose.pose.pose.orientation = orientation_msg;

	// Fill covariance. Order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	pose.pose.covariance.assign(0.0); // Generally uncorrelated
	for (int i=0; i<36; i+=7)
		pose.pose.covariance.elems[i] = 999;
	pose.pose.covariance.elems[0] = uncertainty_fixed; // x
	pose.pose.covariance.elems[7] = uncertainty_fixed; // y

	// ROS subs, pubs
	ros::Subscriber encoder_sub = n.subscribe("/motor_encoder", 100, encoder_callback);
	pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("motor_odom", 50);
	encoder_publisher = n.advertise<std_msgs::Int32>("motor_encoder_abs", 50);

	// Spin until node close
	ros::spin();

	return 0;
}
