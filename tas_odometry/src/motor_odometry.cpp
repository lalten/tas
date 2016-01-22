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
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <tas_odometry/Encoder.h>

ros::Publisher pose_publisher;
ros::Publisher encoder_publisher;

geometry_msgs::TwistWithCovarianceStamped twist;
int32_t encoder_abs = 0;

std::string frame_id;
double ticks_per_meter;
double uncertainty_fixed;
ros::Duration time_offset;

// We get new encoder values here
void encoder_callback(const tas_odometry::Encoder::ConstPtr& encoder_data) {

	// On the first message, save time offset so later timestamps are correct
//	if(time_offset.isZero()) {
//		time_offset = ros::Time::now() - encoder_data->header.stamp;
//	}

	// Build twist msg header
	twist.header.frame_id = frame_id;
	twist.header.stamp = encoder_data->header.stamp + time_offset;
	twist.header.seq = encoder_data->header.seq;

	// save time of message arrival
	double time_now = twist.header.stamp.toSec();

	// Publish integrated/absolute encoder value
	encoder_abs += encoder_data->encoder_ticks;
	std_msgs::Int32 msg;
	msg.data = encoder_abs;
	encoder_publisher.publish(msg);

	// Convert length to meter
	double change_meter = (double) encoder_data->encoder_ticks / ticks_per_meter;
	// Convert time to seconds
	double change_time = (double) encoder_data->duration / 1e6;
	// Calc velocity
	double vel = change_meter / change_time;

	// Add it to message
	twist.twist.twist.linear.x = vel;

	// Send out message
	pose_publisher.publish(twist);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_odometry");
	ros::NodeHandle n;

	// ROS params
	n.param<double>("ticks_per_meter", ticks_per_meter, 100);
	n.param<std::string>("frame_id", frame_id, "base_link");
	n.param<double>("uncertainty_fixed", uncertainty_fixed, 1e-3);

	// Fill covariance. Order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	twist.twist.covariance.assign(0.0); // Generally uncorrelated
	for (int i=0; i<36; i+=7)
		twist.twist.covariance.elems[i] = 999;
	twist.twist.covariance.elems[0] = uncertainty_fixed; // x

	// ROS subs, pubs
	ros::Subscriber encoder_sub = n.subscribe("/motor_encoder", 100, encoder_callback);
	pose_publisher = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("motor_odom", 50);
	encoder_publisher = n.advertise<std_msgs::Int32>("motor_encoder_abs", 50);

	// Spin until node close
	ros::spin();

	return 0;
}
