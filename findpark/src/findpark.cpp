#include "findpark.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/********************************
 * Quirin Körner    -   ga34diz *
 * ******************************/

/// \brief TAS: ROS node to find a Parking spot at wall.
/// \brief Uses matching algorithm. The template image is turned, till the best match is found.

/// \author Quirin Körner: ga34diz
/// \date late 2015
/// \warning beta

/** This ROS node finds a parking Spot in a precreated map*/

/** General information: You need the map before, it's not on the fly  */

using namespace cv;
using namespace std;

//+++++++++ STATIC Predefine ++++++++++++++++//
static bool TESTMAP=true; //if use TEST map 1, if real 0
static bool DISPLAY=true; //if use images should be displayed 1, if not 0
static int STEPSIZE = 1; //deegree step size per matching

static int DISTANCE_X = +15; // - means left - look in the yaml of map
static int DISTANCE_Y = -50; // - means up - look in the yamel of map


//++++++++++++ VAR +++++++++++++//






int main(int argc, char** argv) {

    /// \brief Main function to handle matching process. argv[1] is the path to the map folder. see cmake for more details.

    ros::init(argc, argv, "findpark");
    ros::NodeHandle n;
    MoveBaseClient ac("move_base", true);





    //Subscriber

    //Publisher
    //wii_communication_pub = nh_.advertise<std_msgs::Int16MultiArray>("wii_communication",1);


    tasMapPath = string(argv[1]);

    load_map(); //Loads real and test map

    if(!TESTMAP) {
        //display_mapReal();
        rotate_template(mapReal);
    }

    if(TESTMAP) {
        //display_mapTest();
        rotate_template(mapTest);
    }

    ac.sendGoal(goal);



    return 0;





}




void load_map()
{

    /// \author Quirin Körner: ga34diz


    //load real map image

    if(!TESTMAP)
    {
        //+++++ Try to load real map
        //+++++

        string tasRealMapPath = tasMapPath.append("hectormap.pgm");
        tasRealMapPath.append("hectormap.pgm");

        mapReal = imread(tasRealMapPath.c_str(), CV_LOAD_IMAGE_GRAYSCALE); // Read the map
        if( ! mapReal.data ) //ceck if worked
        {
            ROS_WARN("Could NOT load REAL map. Did you create a map? Is it saved to: %s ?",tasRealMapPath.c_str());
        }
        else
        {
            ROS_INFO("Real map was loaded. Sucess.");
        }
    }

    //load test iamge

    if(TESTMAP)
    {
        //++++++ Try to load Test map
        //++++++

        string tasTestMapPath = tasMapPath;
        tasTestMapPath.append("static/mapTest.pgm");

        mapTest = imread(tasTestMapPath.c_str(), CV_LOAD_IMAGE_GRAYSCALE); //Load map
        if( ! mapTest.data ) //ceck if worked
        {
            ROS_WARN("Could NOT load TEST map. Should not happen, as it is included in the package. Put it in: %s",tasTestMapPath.c_str());
        }
        else
        {
            ROS_INFO("Test map was loaded.");
        }
    }


    //+++++ load image of parking spots
    //++++++
    string tasParkingImgPath = tasMapPath;
    tasParkingImgPath.append("static/mapParkingLeft.pgm");

    mapParkingLeft = imread(tasParkingImgPath.c_str(), CV_LOAD_IMAGE_GRAYSCALE); //Load map
    if( ! mapParkingLeft.data ) //check if worked
    {
        ROS_WARN("Could NOT load matching image for ParkingLeft. Should not happen, as it is included in the package. Put it in: %s", tasParkingImgPath.c_str());
    }
    else
    {
        ROS_INFO("The matching image for ParkingLeft loaded. Sucess.");
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

        //waitKey(0); // Wait for key
    }



}

void display_mapTest()
{
    /// \author Quirin Körner: ga34diz
    /// This function displayes the Test map, if testmode is set.

    if( ! mapTest.data ) //ceck if worked
    {
        ROS_WARN("Test map not loaded, can't display.");
    }
    else
    {
        namedWindow( "map - TAS-Team-06", WINDOW_NORMAL ); // Create window
        resizeWindow("map - TAS-Team-06", 512, 512);
        imshow( "map - TAS-Team-06", mapTest );  // Show map

        //waitKey(0); // Wait for key
    }

}

void display_mapMatchParking()
{
    /// \author Quirin Körner: ga34diz
    /// This function displayes the mapMatchParking image.



    if( ! mapParkingLeft.data ) //ceck if worked
    {
        ROS_WARN("TmapMatchParking not loaded, can't display.");
    }
    else
    {
        namedWindow( "mapMatchParking - TAS-Team-06", WINDOW_NORMAL ); // Create window
        resizeWindow("mapMatchParking - TAS-Team-06", 512, 512);
        imshow( "mapMatchParking - TAS-Team-06", mapParkingLeft );  // Show map

        waitKey(0); // Wait for key
    }

}

void find_spot(cv::Mat map, cv::Mat templateImg)
{
    /// \author Quirin Körner: ga34diz
    /// This function matches the template to the map. The higher the value in result(Mat), the better the match
    /// finds all parking spots of the input "map"

    Mat result; //Matrix with results of matching alogorithm
    int methode = CV_TM_CCOEFF; //matching methode

    matchTemplate(map, templateImg, result, methode); //matching algorithm

    if(false)
    {
        namedWindow( "result Matching - TAS-Team-06", WINDOW_NORMAL ); // Create window
        resizeWindow("result Matching - TAS-Team-06", 512, 512);
        imshow( "result Matching - TAS-Team-06", result );  // Show map
        waitKey(0); // Wait for key
    }

    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() ); //find Maximum in result map

    matchLoc = maxLoc; //if you change matchin mehode, you might need to change this as well
    int matchVal = maxVal;


    //prepare the result of matching to save it in possiblePositions
    vector<int>temp(3,0);
    temp.at(0)=matchLoc.x; //x value on image (pixel)
    temp.at(1)=matchLoc.y; //y value
    temp.at(2)=matchVal; // Value of highest match in this run
    possiblePositions.push_back(temp); //write it to global






}

void rotate_template(cv::Mat map)
{
    ROS_INFO("TEMPLATE MATCHING STARTED. PLEASE WAIT. MAY TAKE MINUTES");

    //we have to rotate the template to match it to the map

    for(int i=0; i<30; i=i+STEPSIZE)
    {
        //rotate template process
        Mat tempTemplate = mapParkingLeft;
        Mat rotateMat;//( 2, 3, CV_32FC1 ), rotation matrix
        Mat rotatedTemplate = Mat::zeros( tempTemplate.rows, tempTemplate.cols, tempTemplate.type() );

        rotateMat = getRotationMatrix2D(Point( tempTemplate.cols/2, tempTemplate.rows/2 ),(-1)*i,1);
        warpAffine(tempTemplate, rotatedTemplate, rotateMat, tempTemplate.size());



        if(false) //show rotated template
        {
            namedWindow( "TemplateImg - TAS-Team-06", WINDOW_NORMAL ); // Create window
            resizeWindow("TemplateImg - TAS-Team-06", 512, 512);
            imshow("TemplateImg - TAS-Team-06", rotatedTemplate );
            waitKey(0); // Wait for key
        }


        find_spot(map, rotatedTemplate);

        if(false) //show results of matching process
        {
            Mat img_display;
            map.copyTo( img_display );
            std::vector<int> temp = possiblePositions.at(i/STEPSIZE);
            rectangle( img_display, Point(temp.at(0), temp.at(1) ), Point( temp.at(0) + rotatedTemplate.cols , temp.at(1) + rotatedTemplate.rows ), Scalar::all(0), 2, 8, 0 );


            namedWindow( "ResultImg - TAS-Team-06", WINDOW_NORMAL ); // Create window
            resizeWindow("ResultImg - TAS-Team-06", 512, 512);
            imshow("ResultImg - TAS-Team-06", img_display );
            waitKey(0); // Wait for key

        }

        if(MaxValIndex < possiblePositions.at(i/STEPSIZE).at(2))
        {
            ROS_INFO("BETTER MATCH FOUND. ANGLE = %d",i*STEPSIZE);
            maxIndex = i;
            MaxValIndex = possiblePositions.at(i/STEPSIZE).at(2);

        }

        ROS_INFO("Point: %d %d %d",possiblePositions.at(i/STEPSIZE).at(0), possiblePositions.at(i/STEPSIZE).at(1),possiblePositions.at(i/STEPSIZE).at(2));

    }

    //calculate start position
    /// the start position has to be calculated, as the template is turned an
    /// therefore the starting position has to be alligned

    double rotationAngle = ((maxIndex*STEPSIZE))*3.14/180;
    int y1 =  sin(rotationAngle)*DISTANCE_X;
    int x1 =  cos(rotationAngle)*DISTANCE_X;
    int y2 =  cos(rotationAngle)*DISTANCE_Y;
    int x2 =  -sin(rotationAngle)*DISTANCE_Y;

    startPositionInMap.x = (x1 + x2) + possiblePositions.at(maxIndex).at(0);
    startPositionInMap.y = (y1 + y2) + possiblePositions.at(maxIndex).at(1);


    ROS_INFO("Best match at: X: %d Y:%d VALUE: %d",possiblePositions.at(maxIndex).at(0), possiblePositions.at(maxIndex).at(1),possiblePositions.at(maxIndex).at(2));
    ROS_INFO("ANGLE MAP<->TEMPLATE = %lf",rotationAngle);


    if(DISPLAY)
    {
        Mat img_display;  //create Mat image to display
        cvtColor( map, img_display, CV_GRAY2BGR );
        //map.copyTo( img_display ); //copy original map to the desplay mat
        std::vector<int> temp = possiblePositions.at(maxIndex); //copy best result to temp

        cv::Point2f offset((cos(rotationAngle)*(mapParkingLeft.cols/2)-sin(rotationAngle)*(mapParkingLeft.rows/2)),(sin(rotationAngle)*(mapParkingLeft.cols/2)+cos(rotationAngle)*(mapParkingLeft.rows/2)));
        // create rotated rectangle as overlay
        cv::Point corner(
                    possiblePositions.at(maxIndex).at(0) + offset.x    //left top corner of match
                    //+ cos(rotationAngle)*(mapParkingLeft.cols/2)- //half picture size roatated
                    //sin(rotationAngle)*(mapParkingLeft.rows/2)  //half picture size but roated
                    ,
                    possiblePositions.at(maxIndex).at(1) + offset.y     //left top corner of match
                    //+ sin(rotationAngle)*(mapParkingLeft.cols/2)+ //half picture size roatated
                    //cos(rotationAngle)*(mapParkingLeft.rows/2)    //half picture size but roated

                    );

        cv::RotatedRect rRect = RotatedRect(corner, mapParkingLeft.size(), rotationAngle*180/3.14);
        Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            line(img_display, vertices[i], vertices[(i+1)%4], Scalar(0,0,255));
        }


        // paint starting point to map

        // circle(img_display, Point(temp.at(0), temp.at(1)), 3,1,3);

        circle(img_display, startPositionInMap, 3,Scalar(0,0,255),3);
        cv::Point tempOffset(-x2,-y2);
        line(img_display,startPositionInMap,startPositionInMap + tempOffset,Scalar(255,0,0));

        namedWindow( "ResultImg - TAS-Team-06", WINDOW_NORMAL ); // Create window
        resizeWindow("ResultImg - TAS-Team-06", 850, 850);
        imshow("ResultImg - TAS-Team-06", img_display );
        waitKey(0); // Wait for key

    }

    calculate_start(startPositionInMap.x, startPositionInMap.y, rotationAngle);


}

bool calculate_start(int start_x, int start_y, double orientation)
{
    /// \author Quirin Körner: ga34diz
    /// This function calculates the start position for the parking procedure of the parking node.
    /// As Input the X and Y coordinate of the aprking spot are needed


    double mapStart_x;
    double mapStart_y;
    double mapOrientation;

    mapStart_x = start_x;
    mapStart_y = start_y;
    mapOrientation = orientation+3.14;


    sendStartPos(mapStart_x, mapStart_y, mapOrientation);
}

void sendStartPos (double mapStart_x, double mapStart_y, double orientation)
{

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = mapStart_x;
    goal.target_pose.pose.position.y = mapStart_y;
    goal.target_pose.pose.position.z = 0;


    tf::Quaternion temp( tf::Vector3(0,0,1),orientation);
    goal.target_pose.pose.orientation.x = temp.getX();
    goal.target_pose.pose.orientation.y = temp.getY();
    goal.target_pose.pose.orientation.z = temp.getZ();
    goal.target_pose.pose.orientation.w = temp.getW();



}
