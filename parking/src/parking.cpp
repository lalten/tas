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
#define DIST_BOX 0.40  /*!< in cm !! if distance smaller -> say box / not wall */
//distance box <-> startpoint = 29 = 40 (+20 car) - 31 box
//distance wall <-> startpoint = 40 (+20 car)
#define SPEED 38 //20  /*!< Speed in x direction */
#define SPEED_CURVE 21//190 /*!< Speed in curve */
#define TURN 400  /*!< max of y to turn */
#define SEGMENTLEFT 50 /*!< laser segment start value for left look */
#define SEGMENTLEFT_SECOUND 50 /*!< laser segment start value secound box left look */
#define SEGMENT_S 85 /*!< laser segment start value for 30 degree look */
#define SEGMENT_BACK 260 /*!< laser segment start value for back look */
#define DIST_WALL_S_END 0.45 /*!< distance to wall when ending first part of s curve */
#define DIST_BOX_S_END 0.15 /*!< distance to box when ending the whole s curve */
#define DIST_BOX_Mid_END 0.20 /*!< distance to box when stopping the park process */
#define STATE_DELAY 3000000000
#define MAX_DELAY 2000000000

//+++++++++++ STATIC REGLER +++++++++++++++++//
#define NoBoxDistance 0.45 /*!< ###### */
#define SollWallDistance 0.58//0.555 /*!< ###### */
#define SollWallBox 0.27 /*!< ###### */
#define TURN_RATE_REGLER 30 /*!< ###### */
#define FRONT_SEGMENT_END 718
#define DistanceTurnRateRatio 1000;
/* Laser front segments 718, 4 per degree */


//++++++++++++ VAR +++++++++++++//

const bool quirinWillDebug = true; /*!< activated debug message */
const bool quirinWillDebugState = false; /*!< activated debug message */



int numBoxSeen = 0; /*!< number of Boxes seen by car */
int modeStep = 0; /*!< indicates state of statemachine */
bool isBox = false; /*!< flag, which is tree while next to box */
int segmentStart=SEGMENTLEFT;
double laser_Front=0;




int main(int argc, char** argv) {

    ros::init(argc, argv, "parking_listener");

    ros::NodeHandle n;
    ros::init(argc, argv, "parking_sound");
    //sound_play::SoundClient sc;


    //Subscriber
    ros::Subscriber laser_back_sub = n.subscribe("/scan_back", 1000, parkingCallback);
    ros::Subscriber laser_front_sub = n.subscribe("/scan", 1000, scanFrontCallback);

    //Publisher
    parking_servo_publisher = n.advertise<geometry_msgs::Vector3>("servo",1);

    zeit_start = ros::Time::now();


    ros::spin();
}

void scanFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    double temp_laser=0;
    for (int i=FRONT_SEGMENT_END; i>FRONT_SEGMENT_END-SUM_DIST; i--)
    {
        temp_laser = temp_laser + msg->ranges[i];
    }
    laser_Front = temp_laser / SUM_DIST;
    if(quirinWillDebug){ROS_INFO("Laser avarage FRONT to LEFT is:  %lf",laser_Front);}

}


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
        //if(quirinWillDebug){ROS_INFO("DRIVE BY BOXES");}

        //get number of Boxes
        setSpeed(SPEED,calcTurnRate(laser_Front));
        if(getNumBoxes(laser) == 2)
        {
            stopEngine();
            modeStep = 1;
            segmentStart = SEGMENT_S;
            if(quirinWillDebugState){ROS_INFO("FIRST S PART");}
            zeit_start = ros::Time::now();

        }

        break;
    case 1:
         if(quirinWillDebugState){ROS_INFO("FIRST S PART");}

        //START FIRST S
        if(laser <= DIST_WALL_S_END && (ros::Time::now().toNSec() - zeit_start.toNSec() > STATE_DELAY))
        {
            stopEngine();
            modeStep = 2;
            segmentStart = SEGMENT_S;
             if(quirinWillDebug){ROS_INFO("PULL BACK BETWEEN S");}
             zeit_start = ros::Time::now();

        }
        else
        {
            if((ros::Time::now().toNSec() - zeit_start.toNSec()) < STATE_DELAY/4 )
            {
                //setSpeed(-SPEED_CURVE, TURN); //vettel only
            }
            else
            {
                if((ros::Time::now().toNSec() - zeit_start.toNSec()) > STATE_DELAY/2)
                {
                  setSpeed(-SPEED_CURVE, TURN);
                }
                else
                {
                stopEngine();
                }
            }

        }

        break;
    case 2:
         if(quirinWillDebug){ROS_INFO("PULL BACK BETWEEN S");}

        //Pull Back Between S-curve-parts
        stopEngine();
        modeStep = 3;
        segmentStart = SEGMENT_BACK;
        if(quirinWillDebug){ROS_INFO("SECOUND part S curve");}
        zeit_start = ros::Time::now();
        break;
    case 3:
         if(quirinWillDebugState){ROS_INFO("SECOUND part S curve");}

        //secound part s curve

        if(laser <= DIST_BOX_S_END && (ros::Time::now().toNSec() - zeit_start.toNSec() > STATE_DELAY))
        {
            stopEngine();
            modeStep = 4;
            segmentStart = SEGMENT_BACK;
            if(quirinWillDebug){ROS_INFO("GO TO END POSITION");}
            zeit_start = ros::Time::now();
        }
        else
        {
            setSpeed(-SPEED_CURVE, -TURN);
        }



        break;
    case 4:
         if(quirinWillDebugState){ROS_INFO("GO TO END POSITION");}
        //Pull forward for end position
        if(laser >= DIST_BOX_Mid_END || (ros::Time::now().toNSec() - zeit_start.toNSec() > MAX_DELAY))
        {
            stopEngine();
            modeStep = 5;
            segmentStart = SEGMENT_BACK;
            if(quirinWillDebug){ROS_INFO("GOAL RECHEAD. PARRRRRTTTYYYYYYY!");}

        }
        else
        {
            if((ros::Time::now().toNSec() - zeit_start.toNSec() < MAX_DELAY/3*2))
            {
                setSpeed(SPEED+5, TURN);
            }
            else
            {
                setSpeed(SPEED,0);
            }
        }
        break;

    case 5:
         if(quirinWillDebugState){ROS_INFO("GOAL RECHEAD. PARRRRRTTTYYYYYYY!");}

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
            // sound_play::Sound s2 = sc.voiceSound("I see a box!");
        }
    }
    else
    {
        if(isBox)
        {
            //passed box
            isBox=false;
            if(quirinWillDebug){ROS_INFO("Passed by Box, now seeing wall.");}
            // sound_play::Sound s2 = sc.voiceSound("I see a wall!");


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
    servo.y = 1500 + turnOffset; // KONRAD
    //servo.y = turnOffset; // KONRAD
    servo.z = 0;

    parking_servo_publisher.publish(servo);
    //if(quirinWillDebug){ROS_INFO("Move It! speed:  %lf turnRate: %lf",turnRate);}
}

int calcTurnRate (double distance)
{
    int turnRate=0;
    float sollDistance;
    if(distance < NoBoxDistance )
    {
        sollDistance=SollWallBox;
        //BOX
    }
    else
    {
        sollDistance=SollWallDistance;
        //noBox

    }

    if (false) {// KONRAD
        //set turnrate relative to distance
        int temp_turnRate=((distance / sollDistance)-1)*DistanceTurnRateRatio;
        if (distance < sollDistance - 0.005 || distance > sollDistance +0.005)
        {
            //rechts lenken
            turnRate = temp_turnRate;
        }
    } else {
        if (distance > (sollDistance + 0.01))
            turnRate = -400;
        else if (distance < (sollDistance -0.01))
            turnRate = 400;
        else
            turnRate = 0;
    }

    return turnRate;

}
