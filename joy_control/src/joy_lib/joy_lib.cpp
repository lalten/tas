#include "joy_lib.h"

joy_lib::joy_lib() {
    joy_servo_publisher = nh.advertise<geometry_msgs::Vector3>("servo",1);
    joy_communication_publisher = nh.advertise<std_msgs::Int16MultiArray>("wii_communication",1);
    joy_subscriber = nh.subscribe("/joy", 100, &joy_lib::joyStateCallback, this);

    msgInit(joy_state);

    controlMode.data = 0;
    emergencyBrake.data = 1;
}

void joy_lib::joyStateCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    std::vector<float> axes;
    std::vector<int> buttons;

    axes = msg->axes;
    buttons = msg->buttons;

    // START-Button                 || C--> START
    if (buttons[3]==1)
    {
        controlMode.data=1;

        // X-Button                 || Z--> X
        if(buttons[14]==1)
            emergencyBrake.data = 1;
        else
            emergencyBrake.data = 0;
    }
    else
    {
        controlMode.data = 0;

        // X-Button                 || Z--> X
        if(buttons[14]==1){
            emergencyBrake.data = 1;
            servo.x = 1500;
            servo.y = 1500;
        } else {
            emergencyBrake.data = 0;

            //
            if(axse[1]>=0)
                SCALE_FACTOR_THROTTLE = 200;
            else
                SCALE_FACTOR_THROTTLE = 300;

            servo.x = 1500 + SCALE_FACTOR_THROTTLE*axse[1];
            servo.y = 1500 + SCALE_FAKTOR_STEERING*axse[0];
        }

        joy_servo_publisher.publish(servo);
    }

    joy_state.data[0] = controlMode.data;
    joy_state.data[1] = emergencyBrake.data;
}

void joy_lib::msgInit(std_msgs::Int16MultiArray &msg)
{
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 2;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "flag";
    msg.data.clear();
    msg.data.resize(2);
}
