#include "plannerlib.h"
#include <math.h>

plannerLib::plannerLib()
{
    // Subscribe Local Costmap
    //costmap_Sub = nh.subscribe<>("",2,&,this);



    //wii_communication_pub = nh_.advertise<std_msgs::Int16MultiArray>("wii_communication",1);


}

void plannerLib::handleNewCostmap(std::vector<> data)
{
    float<int> costmap = data.;
    int width = data.;
    int hight = data.;
    float res = data.;

    carWidth = CAR_WIDTH / res;
    carLength = CAR_LENGTH / res;



}

/*  CREATE BOX
 *
 *  This Function creates the points of the corners of the car, at a certain point, with a certai angle
 *
 *                                                                                                                      */
void plannerLib::createBox(float positionX, float positionY, float angle, std::vector<float> &xPoints, std::vector<float> &yPoints)
{
    // Create Basic Box
    float p1x = -carWidth/2;
    float p1y = 0.0f;

    float p2x = p1x;
    float p2y = -carLength;

    float p3x = carWidth/2;
    float p3y = p2y;

    float p4x = p3x;
    float p4y = 0.0f;

    // Fill in x Coordinates
    xPoints.clear();
    xPoints.push_back(p1x*cos(angle)-p1y*sin(angle) + positionX); // P1 with angle and reset by given offset from orgin
    xPoints.push_back(p2x*cos(angle)-p2y*sin(angle) + positionX); // P2 with angle and reset by given offset from orgin
    xPoints.push_back(p3x*cos(angle)-p3y*sin(angle) + positionX); // P3 with angle and reset by given offset from orgin
    xPoints.push_back(p4x*cos(angle)-p4y*sin(angle) + positionX); // P4 with angle and reset by given offset from orgin
    //xPoints.push_back(p1x*cos(angle)-p1y*sin(angle));

    // Fill in y Coordinates
    yPoints.clear();
    yPoints.push_back(p1x*sin(angle)+p1y*cos(angle) + positionY); // P1 with angle and reset by given offset from orgin
    yPoints.push_back(p2x*sin(angle)+p2y*cos(angle) + positionY); // P2 with angle and reset by given offset from orgin
    yPoints.push_back(p3x*sin(angle)+p3y*cos(angle) + positionY); // P3 with angle and reset by given offset from orgin
    yPoints.push_back(p4x*sin(angle)+p4y*cos(angle) + positionY); // P4 with angle and reset by given offset from orgin
    //yPoints.push_back(p1x*sin(angle)+p1y*cos(angle));
}

/*  CREATE CALC POINTS
 *
 *  This Function creats a List of points, which are needed to calculate the cost of the current Path
 *
 *                                                                                                                      */
void plannerLib::createCalcPoints(std::vector<float> coordinatsX, std::vector<float> coordinatsY, std::vector<float> &xPoints, std::vector<float> &yPoints)
{
    // There must be an equal number of X and Y Coordinates
    if (coordinatsX.size() != coordinatsY.size()){
        ROS_ERROR("Wrong use of createCalcPoints");
        return;
    }

    xPoints.clear();
    yPoints.clear();

    // Starting at the second point of the path, get all Points for cost calculation
    for (int currentPoint = 1; currentPoint < coordinatsX.size(); currentPoint++)
    {
        float angle;    // Angle of current Point

        // If the X-Value of current and privious point is the same --> angle = 0
        if(coordinatsX.at(currentPoint) - coordinatsX.at(currentPoint-1) == 0)
            angle = 0;

        // If the X-Vaue is left of the previous point
        else if(coordinatsX.at(currentPoint) - coordinatsX.at(currentPoint-1) < 0)
            angle = atan((coordinatsY.at(currentPoint)-coordinatsY.at(currentPoint-1)) /
                         (coordinatsX.at(currentPoint)-coordinatsX.at(currentPoint-1)))
                    + (PI/2);

        // If the X-Vaue is right of the previous point
        else
            angle = atan((coordinatsY.at(currentPoint)-coordinatsY.at(currentPoint-1)) /
                         (coordinatsX.at(currentPoint)-coordinatsX.at(currentPoint-1)))
                    - (PI/2);

        // Get the four corners of the car
        std::vector<float> xPointsCar;
        std::vector<float> yPointsCar;

        createBox(coordinatsX.at(currentPoint),coordinatsY.at(currentPoint),angle,xPointsCar,yPointsCar);

        // Round Points to full number and add to List
        for (int i = 0; i < 4; i++)
        {
            xPoints.push_back(floor(xPointsCar.at(i)+0.5f));
            yPoints.push_back(floor(yPointsCar.at(i)+0.5f));
        }
    }
}

/*  CALCULATE COST
 *
 *  This Function calculates the Costs to the Points out of CREATE CALC POINTS
 * .
 *                                                                                                                      */
float plannerLib::calcCost(std::vector<float> xPoints, std::vector<float> yPoints, std::vector<int> costMap, int width, int hight)
{
    // There must be an equal number of X and Y Coordinates
    if (xPoints.size() != yPoints.size()){
        ROS_ERROR("Wrong use of calcCost");
        return;
    }

    float cost = 0;
    for (int i = 1; i < xPoints.size(); i++)
    {
        // II the calculated point is outside the costmap
        if (xPoints.at(i) < 1 || yPoints.at(i) < 1 | xPoints.at(i) > width | yPoints.at(i) > hight)
            cost += COST_OUTSIDE;
        // ELSE get the cost out of the costmap
        else
            cost += costMap.at(getIndex(xPoints.at(i), yPoints.at(i), width, hight));
    }

    return cost;
}

/*  Calculates the Index for Vector of Costmap to given x and y points
 *
 *                                                                                                                      */
int plannerLib::getIndex(int x, int y, int width, int hight)
{
    //return (y-1)*hight + (x-1)    // IF FLIPPED
    return (x-1)*width + (y-1);
}





