#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <px_comm/OpticalFlowRad.h>
#include <regex>

ros::Publisher twist_publisher;
double fix_covariance;

void flow_callback (const px_comm::OpticalFlowRad::ConstPtr& opt_flow) {

	// Don't publish garbage data
	if(opt_flow->quality == 0){
		return;
	}

	// Build new message from old header
	geometry_msgs::TwistWithCovarianceStamped twist;
	twist.header = opt_flow->header;
	twist.header.stamp = ros::Time::now(); // Otherwise the timestamp on the message is more than the cache length earlier than the newest data in the transform cache
	if(twist.header.frame_id.substr(0,1) == "/")
		twist.header.frame_id = twist.header.frame_id.erase(0,1); // Otherwise: error, because tf2 frame_ids cannot start with a '/'

	// translation from optical flow, in m/s
	twist.twist.twist.linear.x = 1.0e6 * (opt_flow->integrated_x/opt_flow->integration_time_us)/opt_flow->distance;
	twist.twist.twist.linear.y = 1.0e6 * (opt_flow->integrated_y/opt_flow->integration_time_us)/opt_flow->distance;
	twist.twist.twist.linear.z = 0;

	// rotation from integrated gyro, in rad/s
	twist.twist.twist.angular.x = 1.0e6 * opt_flow->integrated_xgyro/opt_flow->integration_time_us;
	twist.twist.twist.angular.y = 1.0e6 * opt_flow->integrated_ygyro/opt_flow->integration_time_us;
	twist.twist.twist.angular.z = 1.0e6 * opt_flow->integrated_zgyro/opt_flow->integration_time_us;

	// Populate covariance matrix with uncertainty values
	double uncertainty = fix_covariance;
	if (fix_covariance == 0) {
	  uncertainty = pow(10, -1.0 * opt_flow->quality / (255.0/6.0));
	}
	twist.twist.covariance.assign(0.0); // We say that generally, our data is uncorrelated to each other
	// However, we have uncertainties for
	// x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
	for (int i=0; i<36; i+=7)
		twist.twist.covariance[i] = uncertainty;

//	ROS_INFO("qual:\t%d, uncert:\t%E\n", opt_flow->quality, uncertainty);

	twist_publisher.publish(twist);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "optflow_odometry");
	ros::NodeHandle n;

	ros::Subscriber flow_subscriber = n.subscribe("/px4flow/opt_flow_rad", 100, flow_callback);

	twist_publisher = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("visual_odom", 50);

  // Option for fixing covariance values (0 means auto-calculate)
  n.param<double>("fix_covariance", fix_covariance, 0.0);

	ros::spin();

	return 0;
}
