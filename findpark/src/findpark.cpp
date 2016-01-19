#include "findpark.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/********************************
 * Quirin Körner    -   ga34diz *
 * ******************************/

/// \brief TAS: ROS node to find a Parking spot on wall
/// \author Quirin Körner: ga34diz
/// \date late 2015
/// \warning beta

/** This ROS node finds a parking Spot in a precreated map*/

/** General information: You need the map before, it's not on the fly  */

using namespace cv;
using namespace std;

//+++++++++ STATIC Predefine ++++++++++++++++//
static bool TESTMAP=true; //if use TEST map 1, if real 0


//++++++++++++ VAR +++++++++++++//






int main(int argc, char** argv) {

    //*************** not in use!
    /*
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
    */


    load_map(); //Loads real and test map

    if(!TESTMAP) { display_mapReal(); }

    if(TESTMAP) { display_mapTest(); }

    return 0;



}



void load_map()
{

    /// \author Quirin Körner: ga34diz


    if(!TESTMAP)
    {
    //+++++ Try to load real map
    //+++++
    string mapRealPath("../../../tas/launch/config/map_server/hectormap.pgm"); //path to real map
    mapReal = imread(mapRealPath.c_str(), CV_LOAD_IMAGE_GRAYSCALE); // Read the map
    if( ! mapReal.data ) //ceck if worked
    {
        ROS_WARN("Could NOT load REAL map");
    }
    else
    {
        ROS_INFO("Real map was loaded. Sucess.");
    }
    }

    if(TESTMAP)
    {
    //++++++ Try to load Test map
    //++++++
    string mapTestPath("/home/quirin/Downloads/mapTest.pgm"); //path to test map
    mapTest = imread(mapTestPath.c_str(), CV_LOAD_IMAGE_GRAYSCALE); //Load map
    if( ! mapTest.data ) //ceck if worked
    {
        ROS_WARN("Could NOT load TEST map. Should not happen, as it is included in the package. Put it in: /home/quirin/Downloads/mapTest.pgm");
    }
    else
    {
        ROS_INFO("Test map was loaded. Sucess.");
    }
    }


}

void display_mapReal()
{

    /// \author Quirin Körner: ga34diz


    if( ! mapReal.data ) //check if worked
    {
        ROS_WARN("Real map not loaded, can't display");
    }
    else
    {
        namedWindow( "real map - TAS-Team-06", WINDOW_NORMAL ); // Create window
        resizeWindow("eal map - TAS-Team-06", 512, 512);
        imshow( "real map - TAS-Team-06", mapReal );  // Show map

        waitKey(0); // Wait for key
    }



}

void display_mapTest()
{
    /// \author Quirin Körner: ga34diz

    if( ! mapTest.data ) //ceck if worked
    {
        ROS_WARN("Test map not loaded, can't display.");
    }
    else
    {
    namedWindow( "test map - TAS-Team-06", WINDOW_NORMAL ); // Create window
    resizeWindow("test map - TAS-Team-06", 512, 512);
    imshow( "test map - TAS-Team-06", mapTest );  // Show map

    waitKey(0); // Wait for key
    }

}
