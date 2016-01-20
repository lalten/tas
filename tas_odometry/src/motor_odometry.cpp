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

ros::Publisher pose_publisher;
ros::Publisher encoder_publisher;

geometry_msgs::TwistWithCovarianceStamped twist;
uint32_t seq = 0;
int32_t encoder_abs = 0;

std::string frame_id;
double ticks_per_meter;
double uncertainty_fixed;
double last_change_timestamp;
double vel_now = 0;
double acc_now = 0;
double extrapolated_stopping_time = std::numeric_limits<double>::infinity();

// We get new encoder values here
void encoder_callback(const std_msgs::Int32::ConstPtr& encoder_change) {

	// Update header
	twist.header.frame_id = frame_id;
	twist.header.stamp = ros::Time::now();
	twist.header.seq = seq++;

	// save time of message arrival
	double time_now = twist.header.stamp.toSec();

	// Publish integrated/absolute encoder value
	encoder_abs += encoder_change->data;
	std_msgs::Int32 msg;
	msg.data = encoder_abs;
	encoder_publisher.publish(msg);

	// Convert length to meter
	double change_meter = (double) encoder_change->data / ticks_per_meter;

	// If encoder value did change, compute current velocity
	double time_diff = time_now - last_change_timestamp;
	if(change_meter != 0 && time_diff != 0) {
		last_change_timestamp = time_now;
		double vel_last = vel_now;
		vel_now = change_meter / time_diff;
		acc_now = (vel_last - vel_now) / time_diff;

		// if slowing down, linearly extrapolate stop time
		if ( (acc_now > 0 && vel_now < 0) || (acc_now > 0 && vel_now > 0) ) {
			extrapolated_stopping_time = - vel_now/acc_now + time_now;
		} else {
			// we're accelerating, never going to stop! (maybe ;) )
			extrapolated_stopping_time = std::numeric_limits<double>::infinity();
		}
	}
	// If we should have stopped by now (we can't really know, as this doesn't create ticks), set v=0
	else if (time_now >= extrapolated_stopping_time) {
		vel_now = 0;
	}

	// Add it to message
	twist.twist.twist.linear.x = vel_now;

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

	// Initialize some other values
	last_change_timestamp = ros::Time::now().toSec();

	// ROS subs, pubs
	ros::Subscriber encoder_sub = n.subscribe("/motor_encoder", 100, encoder_callback);
	pose_publisher = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("motor_odom", 50);
	encoder_publisher = n.advertise<std_msgs::Int32>("motor_encoder_abs", 50);

	// Spin until node close
	ros::spin();

	return 0;
}
