#include "wii_lib.h"

#include <iostream>
#include <fstream>

using namespace std;

wii_lib::wii_lib()
{
    /*Initilaization of publishers, subscribers and messages*/
    wii_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    wii_communication_pub = nh_.advertise<std_msgs::Int16MultiArray>("wii_communication",1);

    wii_sub_ = nh_.subscribe<wiimote::State>("wiimote/state",100,&wii_lib::wiiStateCallback,this);


    msg_Initialization(wii_state_);

    sizeOfWaypointsList = 0;


    controlMode.data = 0;
    emergencyBrake.data = 1;
}

void wii_lib::wiiStateCallback(const wiimote::State::ConstPtr& wiiState)
{
    /*check if C button is pressed*/
    if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_C]==1)
    {
        controlMode.data = 1; /*setting controlMode flag to 1*/

        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            emergencyBrake.data = 1; /*setting emergencyBrake flag to 1*/
        }
        else
        {
            emergencyBrake.data = 0; /*setting emergencyBrake flag to 0*/
        }
    }
    else
    {
        controlMode.data = 0;/*setting controlMode flag to 0*/

        /*check if Z button is pressed*/
        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            ROS_DEBUG("HINWEIS: KONRAD QUIRIN STELLE");
            addCurrentWayPoint();
            saveWayPointFile();
            emergencyBrake.data = 1; /*setting emergencyBrake flag to 1*/

            servo.x = 1500;
            servo.y = 1500;
        }
        else
        {
            emergencyBrake.data = 0; /*setting emergencyBrake flag to 0*/

            if(wiiState.get()->nunchuk_joystick_zeroed[1]>=0)
            {
                SCALE_FAKTOR_THROTTLE = 200; /*scale factor for driving forward*/
            }
            else
            {
                SCALE_FAKTOR_THROTTLE = 300; /*scale factor for driving reverse*/
            }

            /* mapping analog nunchuk state to servo command*/
            servo.x = 1500 + SCALE_FAKTOR_THROTTLE*wiiState.get()->nunchuk_joystick_zeroed[1];
            servo.y = 1500 + SCALE_FAKTOR_STEERING*wiiState.get()->nunchuk_joystick_zeroed[0];
        }

        wii_servo_pub_.publish(servo); /*publish servo messages to arduino*/
    }



    wii_state_.data[0] = controlMode.data;
    wii_state_.data[1] = emergencyBrake.data;
}

void wii_lib::setCurrentPos()
{

    tf::StampedTransform transform;

    const std::string a = "map";
    const std::string b = "base_link";

    try{
        tf_map_baselink.lookupTransform(a, b, ros::Time(0), transform);
    } catch (tf::TransformException e) {
        ROS_DEBUG("TransformException");
    }

    /*
    currentWayPoint.position.x = transform.getOrigin().x();
    currentWayPoint.position.y = transform.getOrigin().y();
    currentWayPoint.position.z = 0;
    currentWayPoint.orientation.x = 0.000;
    currentWayPoint.orientation.y = 0.000;
    currentWayPoint.orientation.z = -0.76;
    currentWayPoint.orientation.w = 0.64;
    */
}

void wii_lib::addCurrentWayPoint()
{
    setCurrentPos();
    sizeOfWaypointsList++;
    waypoints.push_back(currentWayPoint);

}

void wii_lib::saveWayPointFile()
{
    ofstream myfile;
    myfile.open ("TAS_GROUP_06-Waypoints.txt");
    myfile << "NumbWayPoints:" << sizeOfWaypointsList << "\n";
    for (int i = 0; i < sizeOfWaypointsList; i++){
        myfile << "Pos:" << waypoints.at(i).position.x;
        myfile << ":" << waypoints.at(i).position.y << "\n";
        myfile << "Orient:" << waypoints.at(i).orientation.z;
        myfile << ":" << waypoints.at(i).orientation.w << "\n";
    }
    myfile.close();
}

void wii_lib::msg_Initialization(std_msgs::Int16MultiArray &msg)
{
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 2;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "flag";
    msg.data.clear();
    msg.data.resize(2);
}

