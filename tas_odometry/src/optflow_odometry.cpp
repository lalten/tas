#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <px_comm/OpticalFlowRad.h>

ros::Publisher twist_publisher;
tf::TransformBroadcaster* odom_broadcaster;
std::string odom_frame;
std::string base_frame;
bool pub_tf;


void flow_callback (const px_comm::OpticalFlowRad::ConstPtr& opt_flow) {

	// Don't publish garbage data
	if(opt_flow->quality == 0){
		return;
	}

	geometry_msgs::TwistWithCovarianceStamped twist;
	twist.header = opt_flow->header;

	// translation from optical flow, in m/s
	twist.twist.twist.linear.x = (opt_flow->integrated_x/opt_flow->integration_time_us)/opt_flow->distance;
	twist.twist.twist.linear.y = (opt_flow->integrated_y/opt_flow->integration_time_us)/opt_flow->distance;
	twist.twist.twist.linear.z = 0;

	// rotation from integrated gyro, in rad/s
	twist.twist.twist.angular.x = opt_flow->integrated_xgyro/opt_flow->integration_time_us;
	twist.twist.twist.angular.y = opt_flow->integrated_ygyro/opt_flow->integration_time_us;
	twist.twist.twist.angular.z = opt_flow->integrated_zgyro/opt_flow->integration_time_us;

	// TODO: use opt_flow->quality for this
	//# Row-major representation of the 6x6 covariance matrix
	//# The orientation parameters use a fixed-axis representation.
	//# In order, the parameters are:
	//# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	twist.twist.covariance.assign(0.0);

	twist_publisher.publish(twist);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "optflow_odometry");
	ros::NodeHandle n;

	ros::Subscriber flow_subscriber = n.subscribe("/px4flow/opt_flow_rad", 100, flow_callback);

	twist_publisher = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("visual_odom", 50);

	ros::spin();

	return 0;
}
