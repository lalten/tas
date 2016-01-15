/* Motor Odometry
 *
 * This node reads in encoder data from an Arduino monitoring a sensored
 * brushless motor. It uses IMU orientation data to append (in 2D or 3D) the
 * length driven according to the motor encoders.
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

ros::Publisher pose_publisher;
ros::Publisher encoder_publisher;

tf::Quaternion orientation = tf::Quaternion(0,0,0,1);
tf::Point position;
uint32_t seq = 0;
int32_t encoder_abs = 0;

std::string frame_id;
double ticks_per_meter;
double uncertainty_fixed;
bool mode_2d;

// We get new orientation values from the IMU here
void imu_callback (const sensor_msgs::ImuConstPtr &imu_msg) {
	tf::quaternionMsgToTF(imu_msg->orientation, orientation);
}

// We get new encoder values here
void encoder_callback(const std_msgs::Int32::ConstPtr& encoder_change) {

	// Publish integrated/absolute encoder value
	encoder_abs += encoder_change->data;
	std_msgs::Int32 msg;
	msg.data = encoder_abs;
	encoder_publisher.publish(msg);

	// Convert length to meter
	double change_meter = (double) encoder_change->data / ticks_per_meter;

	// Create normalized orientation vector from IMU quaternion
	tf::Vector3 orientation_vect = orientation.getAxis();
	if(mode_2d) orientation_vect.setZ(0.0);
	orientation_vect.normalize();

	// Note that extrapolating current orientation from orientation and
	// rotational velocities might be more accurate. But this should be good
	// enough, and wheel slip etc. makes the odometry drift anyways.

	// Scale vector to distance driven and add it to position
	position += orientation_vect * change_meter;

	// Build message and header
	geometry_msgs::PoseWithCovarianceStamped pose;
	pose.header.frame_id = frame_id;
	pose.header.stamp = ros::Time::now();
	pose.header.seq = seq++;

	// Fill covariance. Order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	pose.pose.covariance.assign(0.0); // Generally uncorrelated
	for (int i=0; i<36; i+=7)
		pose.pose.covariance.elems[i] = uncertainty_fixed;

	// Convert and set position and orientation
	geometry_msgs::Quaternion orientation_msg;
	geometry_msgs::Point position_msg;
	tf::quaternionTFToMsg(orientation, orientation_msg);
	tf::pointTFToMsg(position, position_msg);
	pose.pose.pose.position = position_msg;
	pose.pose.pose.orientation = orientation_msg;

	// Send out message
	pose_publisher.publish(pose);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_odometry");
	ros::NodeHandle n;

	n.param<double>("ticks_per_meter", ticks_per_meter, 1e3); // TODO: measure this
	n.param<std::string>("frame_id", frame_id, "base_motor");
	n.param<double>("uncertainty_fixed", uncertainty_fixed, 1e-3);
	n.param<bool>("mode_2d", mode_2d, true);

	ros::Subscriber encoder_sub = n.subscribe("/motor_encoder", 100, encoder_callback);
	ros::Subscriber imu_sub = n.subscribe("/imu/data", 100, imu_callback);
	pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("motor_odom", 50);
	encoder_publisher = n.advertise<std_msgs::Int32>("motor_encoder_abs", 50);

	ros::spin();

	return 0;
}
