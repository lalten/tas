#ifndef PLANNERLIB_H
#define PLANNERLIB_H

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>


#define COST_OUTSIDE    50      // Sets the cost out of the Costmap to 50
#define CAR_WIDTH       0.45    // Car Width in Meter
#define CAR_LENGTH      0.70    // Car Length in Meter
#define PATH_LENGTH     3.00    // Length of the global Path in Meter
#define THRESHOLD       100     // Cost-Threshold when to calculate alternative Paths
#define MAX_ANGLE       36      // Maximal Angle in Degree
#define RESOLUTION      2       // Angle Resolution in Degree
#define PI              3.1415926

class plannerLib
{
public:
    plannerLib();

    ros::NodeHandle nh;
    ros::Subscriber costmap_Sub, globalPath_Sub;
    ros::Publisher path_Pub;

    void refreshGlobalPosition();


private:

    float globalCoords[3];
    float carWidth;             // In Pixel !!!
    float carLength;
    std::vector<float> originalPathX;
    std::vector<float> originalPathY;
    std::vector< std::vector<float> > bestPath;
    std::vector< std::vector<float> > restPath;
    nav_msgs::Path ownPath;


    void handleNewCostmap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // This Function creates the points of the corners of the car, at a certain point, with a certai angle
    void createBox(float positionX, float positionY, float angle,                           // INPUT:  Position X, Position Y, Angle
                   std::vector<float> &xPoints, std::vector<float> &yPoints);               // OUTPUT: XCoordinates of the Points, YCoordinates of the Points

    // This Function returns all Points whcih are used to calculate the cost of a Path
    void createCalcPoints(std::vector<float> coordinatsX, std::vector<float> coordinatsY,   // INPUT: VectorPosition of Path X, VectorPosition of Path Y
                   std::vector<int> &xPoints, std::vector<int> &yPoints);                   // OUTPUT: XCoordinates of the Points, YCoordinates of the Points

    // This Function calculates the Costs to the Points out of CREATE CALC POINTS
    float calcCost(std::vector<int> xPoints, std::vector<int> yPoints,                      // INPUT: VectorPoints of Path X, VectorPoints of Path y
                   std::vector<int> costMap, int width, int hight);                         // INPUT: Vector of Costmap, width of costmap, hight of costmap

    // This Function creats a List of points, which are needed to calculate the cost of the current Path
    void createArtificialPath(std::vector<float> startPoint, std::vector<float> endPoint,   // INPUT: Startpoint as Vector, Endpoint as Vector
                              int steps,                                                    // INPUT: How many steps for the path
                              std::vector<float> &newPathX, std::vector<float> &newPathY);  // OUTPUT: new linar Path with # steps from start to endpoint

    // Calculates the Index for Vector of Costmap to given x and y points
    int getIndex(int x, int y, int width, int hight);

    // Provides the Path to a structure used by the bestpath
    std::vector< std::vector<float> > convertToBestPath(std::vector<float> xVals, std::vector<float> yVals);




    void refreshGlobalPath(const nav_msgs::Path::ConstPtr& path);

    // HELPER FUNCTIONS
    float rad(float deg);
    int getPt_bezier(int n1, int n2, float perc);

};

#endif
