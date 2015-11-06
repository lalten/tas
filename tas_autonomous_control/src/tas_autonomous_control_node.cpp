#include "control/control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);

    int controlModeLast = autonomous_control.control_Mode.data;

    while(ros::ok())
    {
        if(controlModeLast != 0 && autonomous_control.control_Mode.data==0)
        {
        	controlModeLast = autonomous_control.control_Mode.data;
            ROS_INFO("Switching to manual control mode");
        }

        if(autonomous_control.control_Mode.data != 0)
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                if(controlModeLast == 0)
				{
                    ROS_INFO("Switching to automatic control mode");
                	controlModeLast = autonomous_control.control_Mode.data;
				}
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1550;
                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    autonomous_control.control_servo.x = 1300;
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }



        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
