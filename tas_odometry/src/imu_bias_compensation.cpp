/* IMU bias compensation
 *
 * This node reads in data from a IMU topic. In its internal state machine's
 * first state, it calculates the average over the first 1000 samples. In the
 * second state, it subtracts this average from later samples, effectively
 * removing gravity and orientation-induced acceleration.
 *
 * ROS input topics:
 * /imu/data - IMU sensor data
 *
 * ROS output topics:
 * /imu/calibrated - average-compensated IMU data
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imu_publisher;

enum class State {RECORD, COMPENSATE};

State state = State::RECORD;

sensor_msgs::Imu bias; // defaults to zeros
unsigned long num_msgs = 0;

static const unsigned long max_msgs = 1000;

void imu_callback (const sensor_msgs::ImuConstPtr imu_msg) {

	switch (state) {
	case State::RECORD:
	{
		bias.angular_velocity.x += imu_msg->angular_velocity.x;
		bias.angular_velocity.y += imu_msg->angular_velocity.y;
		bias.angular_velocity.z += imu_msg->angular_velocity.z;
		bias.linear_acceleration.x += imu_msg->linear_acceleration.x;
		bias.linear_acceleration.y += imu_msg->linear_acceleration.y;
		bias.linear_acceleration.z += imu_msg->linear_acceleration.z;
		num_msgs++;

		if(num_msgs == max_msgs)
		{
			bias.angular_velocity.x /= num_msgs;
			bias.angular_velocity.y /= num_msgs;
			bias.angular_velocity.z /= num_msgs;
			bias.linear_acceleration.x /= num_msgs;
			bias.linear_acceleration.y /= num_msgs;
			bias.linear_acceleration.z /= num_msgs;
			state = State::COMPENSATE;

			ROS_INFO("Imu calibration done (ang %.3lf/%.3lf/%.3lf, lin %.3lf/%.3lf/%.3lf)",
					bias.angular_velocity.x, bias.angular_velocity.y, bias.angular_velocity.z,
					bias.linear_acceleration.x, bias.linear_acceleration.y, bias.linear_acceleration.z
					);
		}
		break;
	}
	case State::COMPENSATE:
	{
		sensor_msgs::Imu imu_msg_comp = *imu_msg;
		imu_msg_comp.angular_velocity.x -= bias.angular_velocity.x;
		imu_msg_comp.angular_velocity.y -= bias.angular_velocity.y;
		imu_msg_comp.angular_velocity.z -= bias.angular_velocity.z;
		imu_msg_comp.linear_acceleration.x -= bias.linear_acceleration.x;
		imu_msg_comp.linear_acceleration.y -= bias.linear_acceleration.y;
		imu_msg_comp.linear_acceleration.z -= bias.linear_acceleration.z;

		imu_publisher.publish(imu_msg_comp);
		break;
	}
	}

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "imu_bias_compensation");
	ros::NodeHandle n;

	ros::Subscriber imu_subscriber = n.subscribe("/imu/data", 100, imu_callback);

	imu_publisher = n.advertise<sensor_msgs::Imu>("/imu/compensated", 100, true);

	ros::spin();

	return 0;
}
