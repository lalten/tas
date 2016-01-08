#include "wii_lib.h"

#include <iostream>
#include <fstream>

using namespace std;

///
/// \brief wii_lib::wii_lib
/// Konrad Vowinkel & Quirn Körner


/*To simulate this node without real car do this:
 * Everything with $ in a new terminal window
 *
 * $ roslaunch tas_simulator startSimulation.launch
 *
 * publish the tf, otherwise there it won't find any
 * $ rostopic pub /tf tf/tfMessage
 * press tab tab and edit xyzw value + child (odom) + link (map) -> Enter
 * $ rostopic pub /tf tf/tfMessage
 * press tab tab and edit xazw value + child (odom) + link (base_link) -> Enter
 *
 * $ rosrun wii_control wii_node
 *
 * $ rostopic pub /wiimote/state wiimote/State
 * tab tab set first of N-chuck buttons to true, to publish Z (or secound for C)
 *
*/

///ToDo siehe code + falsche position und winkel übertragen mit echten Auto, keine Ahnung warum
///
// KV:
// Timer zum eintragen der Wegpunkte und zum Übermitteln der Wegpunkte



wii_lib::wii_lib() : ac("move_base", true)
{
    /*Initilaization of publishers, subscribers and messages*/
    wii_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    wii_communication_pub = nh_.advertise<std_msgs::Int16MultiArray>("wii_communication",1);


    wii_sub_ = nh_.subscribe<wiimote::State>("wiimote/state",100,&wii_lib::wiiStateCallback,this);


    msg_Initialization(wii_state_);

    sizeOfWaypointsList = 0;

    lastTime = ros::Time::now().toSec();
    secondsToWait = 3.0;

    controlMode.data = 0;
    emergencyBrake.data = 1;
}

void wii_lib::wiiStateCallback(const wiimote::State::ConstPtr& wiiState)
{
    /*check if C button is pressed*/
    if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_C]==1)
    {
        controlMode.data = 1; /*setting controlMode flag to 1*/

        // Wenn bereits X sec vergangen sind
        if (ros::Time::now().toSec() - lastTime > secondsToWait) {
            ROS_INFO("C Button Pressed: Poses will be transmitted");
            sendList();
            lastTime = ros::Time::now().toSec();
        } else {
            ROS_INFO("Wait X second and press C again...");
        }


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

            if (ros::Time::now().toSec() - lastTime > secondsToWait){
                ROS_INFO("Z Button Pressed: Current Pose will be set");
                setCurrentPos();
                lastTime = ros::Time::now().toSec();
            } else {
                ROS_INFO("Z Button Pressed: Wait X seconds and press Z again...");
            }
            //addCurrentWayPoint();
            //saveWayPointFile();

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
    bool flagErfolg;

    try
    {
        tf_map_baselink.lookupTransform(a, b, ros::Time(0), transform);
        flagErfolg=true;
    }
    catch (tf::TransformException e)
    {
        ROS_WARN("TransformException: TF (map/base_link) published?");
        flagErfolg=false;
    }

    if(flagErfolg)
    {

    //save current Orientation

    tf::Quaternion ausrichtungQ;
    ausrichtungQ = transform.getRotation();


    //get current Position
    tf::Vector3 position;
    position = transform.getOrigin();


    ROS_INFO("Winkel Ausrichtung: %lf", ausrichtungQ.getAngle());
    ROS_INFO("X Position: %lf", position.getX());

    sendGoal(position, ausrichtungQ);

    }
    else
    {
        ROS_WARN("TransformException: No goal to set, because of prior TransformException");
        /// ToDo Was soll jetzt passieren????
    }

}

void wii_lib::addCurrentWayPoint()
{



    /*
    sizeOfWaypointsList++;
    waypoints.push_back(currentWayPoint);
    */

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

void wii_lib::sendGoal(tf::Vector3 position, tf::Quaternion ausrichtungQ)
{


  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = position.getX();
  goal.target_pose.pose.position.y = position.getY();
  goal.target_pose.pose.position.z = position.getZ();
  goal.target_pose.pose.orientation.w = ausrichtungQ.getW();
  tf::Vector3 temp;
  temp = ausrichtungQ.getAxis();
  goal.target_pose.pose.orientation.x = temp.getX();
  goal.target_pose.pose.orientation.y = temp.getY();
  goal.target_pose.pose.orientation.z = temp.getZ();

  if(true)
  {
      //Ziel sofort setzen
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      ROS_INFO("End Sending goal");
  }
  if(true)
  {
      // Ziel im Vector zwischenspeichern

      //Apend msgs to list
      goalList.push_back(goal);
      ROS_INFO("Appended current goal to goalList");
  }



}

void wii_lib::sendList()
{
    std::vector<move_base_msgs::MoveBaseGoal> temp;
    temp = goalList;
    for (int i=0; i<temp.size(); ++i)
    {
        ac.sendGoal(temp.at(i));
        ROS_INFO("Sending goal # %i", i);
    }
}

