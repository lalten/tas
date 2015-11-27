/*!
 ******************************************************************************
 * \brief
 * 	 ROS node, that converts cmd_vel to ackermann_massage
 * \details
 *
 *
 * \author Bernd Kast
 *
 *****************************************************************************/

#include "cmd_vel_ackermann_bridge/cmd_vel_ackermann_bridge.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

CMDVELACKERMANNBRIDGE::CMDVELACKERMANNBRIDGE():
  m_Node("~"),
  m_maxAngle(1.0),
  m_vehicleLength(0.355)
{
    //maxAngle and vehicleLength are configurable by launch files
	m_Node.param("maxAngle", m_maxAngle, m_maxAngle);
	m_Node.param("vehicleLength", m_vehicleLength, m_vehicleLength);
    //publisher and subscribers
	m_ackermann_pub = m_Node.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 10);
	m_cmd_sub = m_Node.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &CMDVELACKERMANNBRIDGE::cmdCallback,this);
}
CMDVELACKERMANNBRIDGE::~CMDVELACKERMANNBRIDGE()
{
}

void CMDVELACKERMANNBRIDGE::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmdVel)
{
	ackermann_msgs::AckermannDriveStamped ackermannMsg;
	ackermannMsg.drive.speed = cmdVel->linear.x;
    //ackermann geometry
	ackermannMsg.drive.steering_angle = atan(cmdVel->angular.z/cmdVel->linear.x*m_vehicleLength);
    //limitation of steering angle
	if(ackermannMsg.drive.steering_angle > m_maxAngle)
	{
		ackermannMsg.drive.steering_angle = m_maxAngle;
	}
	if(ackermannMsg.drive.steering_angle < -m_maxAngle)
	{
		ackermannMsg.drive.steering_angle = -m_maxAngle;
	}
	if(ackermannMsg.drive.steering_angle != ackermannMsg.drive.steering_angle)
	{
		ackermannMsg.drive.steering_angle = 0.0;
	}
    //publish
	m_ackermann_pub.publish(ackermannMsg);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_ackermann_bridge");
	CMDVELACKERMANNBRIDGE bridge;

	ros::spin();
	return 0;
}


