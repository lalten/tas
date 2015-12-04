#include "parking.h"

/********************************
 * Quirin Körner    -   ga34diz *
 * ******************************/

/// \brief TAS: ROS node for parking
/// \author Quirin Körner: ga34diz
/// \date late 2015
/// \warning not doing anything at the moment

/** This ROS node controlls the parking. It needs laser data of the back laser. It sends messages to move the car*/

/** General information: 1 laser segment (back laser) equals x,x°  */

//+++++++++ STATIC VAR ++++++++++++++++//
#define SUM_DIST 10  /*!< number of the first laser segments to sum up */
#define DIST_BOX 0.35  /*!< in cm !! if distance smaller -> say box / not wall */
//distance box <-> startpoint = 29 = 40 (+20 car) - 31 box
//distance wall <-> startpoint = 40 (+20 car)
#define SPEED 20  /*!< Speed in x direction */
#define TURN 400  /*!< max of y to turn */
#define SEGMENTLEFT 0
#define SEGMENT_S 85
#define DIST_WALL_S_END 0.40 /*!< distance to wall when ending first part of s curve */


//++++++++++++ VAR +++++++++++++//
const bool quirinWillDebug = true; /*!< activated debug message */
int numBoxSeen = 0; /*!< number of Boxes seen by car */
int modeStep = 0; /*!< indicates state of statemachine */
bool isBox = false; /*!< flag, which is tree while next to box */
int segmentStart=SEGMENTLEFT;



void parkingCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //Called when new laser data arrives

    //Calculate avarage over laser segments (SUM_DIST) to LEFT
    float laser = 0;
    for (int i=segmentStart; i<SUM_DIST+segmentStart; i++)
    {
        laser = laser + msg->ranges[i];
    }
    laser = laser / SUM_DIST;
    if(quirinWillDebug){ROS_INFO("Laser avarage back to LEFT is:  %lf",laser);}

    //++++++++++ stateMachine +++++++++++++/

    switch (modeStep)
    {
    case 0:

        //get number of Boxes
        setSpeed(SPEED,0);
        if(getNumBoxes(laser) == 2)
        {
            stopEngine();
            modeStep = 1;
            segmentStart = SEGMENT_S;
        }

        break;
    case 1:

        //START FIRST S
        if(laser <= DIST_WALL_S_END)
        {
            stopEngine();
            modeStep = 2;
            segmentStart = SEGMENTLEFT;
        }
        else
        {
            setSpeed(-SPEED*4, TURN);
        }

        break;
    case 2:

        //END FIRST S


        break;
    case 3:

        //PULL FORWARD


        break;
    case 4:

        //GOAL Reached
        stopEngine();


        break;
    default:

        ROS_ERROR("PARKING: unexpected modeStep in statemachine. Stop all engines");
        stopEngine();
        break;
    }


        //+++++++++++ END STATE MACHINE ++++++++++++++++//



}

int main(int argc, char** argv) {

    ros::init(argc, argv, "parking_listener");

    ros::NodeHandle n;

    //Subscriber
    ros::Subscriber laser_back_sub = n.subscribe("/scan_back", 1000, parkingCallback);

    //Publisher
    parking_servo_publisher = n.advertise<geometry_msgs::Vector3>("servo",1);

    ros::spin();
}

bool checkBox (float laser_dist)
{
    bool tempBox = false;
    //check if next to box
    try
    {
        if(laser_dist <= DIST_BOX)
        {
            tempBox = true;
        }
        else
        {
            tempBox = false;
        }
    }
    catch(int e)
    {
        ROS_ERROR("PARKING: error while prozessing laser_data: %i",e);
    }
    return tempBox;
}

int getNumBoxes (float laser_dist)
{
    if(checkBox(laser_dist))
    {
        if(isBox)
        {
            //nothing changed, still near box
        }
        else
        {
            //new Box
            isBox=true;
            numBoxSeen ++;
            if(quirinWillDebug){ROS_INFO("Detected the next Box, Box number: :  %i",numBoxSeen);}
        }
    }
    else
    {
        if(isBox)
        {
            //passed box
            isBox=false;
            if(quirinWillDebug){ROS_INFO("Passed by Box, now seeing wall.");}

        }
        else
        {
            //nothing changed
        }
    }
    return numBoxSeen;
}

void stopEngine()
{
    setSpeed(0,0);
    if(quirinWillDebug){ROS_INFO("Stoped Engine.");}
}
void setSpeed(int speedOffset, int turnOffset)
{
    geometry_msgs::Vector3 servo;
    servo.x = 1500 + speedOffset;
    servo.y = 1500 + turnOffset;
    servo.z = 0;

    parking_servo_publisher.publish(servo);
    if(quirinWillDebug){ROS_INFO("Move It! speed:  %lf",servo.x);}
}
