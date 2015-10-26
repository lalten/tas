#ifndef CMD_VEL_ACKERMANN_BRIDGE_H
#define CMD_VEL_ACKERMANN_BRIDGE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

class CMDVELACKERMANNBRIDGE
{

	public:
		/** Constructor */
		CMDVELACKERMANNBRIDGE();
		/** Destructor */
		~CMDVELACKERMANNBRIDGE();

	private:
		/** ROS node handle */
		ros::NodeHandle m_Node;

		/** cmd_vel subscriber */
		ros::Subscriber m_cmd_sub;

		/** publisher for ackermann */
		ros::Publisher m_ackermann_pub;
		void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmdVel);

		/** length of the vehicle*/
		double m_vehicleLength;
		
		/** maximum of steering angle*/
		double m_maxAngle;
};
#endif
