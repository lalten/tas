#include "plannerlib.h"
#include <math.h>


// Bezierkurve

plannerLib::plannerLib()
{

    // Publish own optimized Path
    path_Pub = nh.advertise<nav_msgs::Path>("ownPath",1);

    // Subscribe the global Path
    globalPath_Sub = nh.subscribe<nav_msgs::Path>("/move_base_node/TrajectoryPlannerROS/global_plan", 100, &plannerLib::refreshGlobalPath,this);

    // Subscribe the local costmap
    costmap_Sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base_node/local_costmap/costmap", 1, &plannerLib::handleNewCostmap, this);

    // PRESETS
    globalCoords[0] = 0;
    globalCoords[1] = 0;
    globalCoords[2] = 0;
}

void plannerLib::refreshGlobalPosition()
{
    tf::TransformListener listener;
    ros::Rate rate(30);

    while(nh.ok())
    {
        tf::StampedTransform transform;

        try {
            listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1).sleep();
            continue;
        }
        globalCoords[0] = (float) transform.getOrigin().x();
        globalCoords[1] = (float) transform.getOrigin().y();
        globalCoords[2] = (float) (transform.getRotation().getAngle()/PI*180.0* transform.getRotation().getAxis().getZ());
        globalCoords[2] = globalCoords[2]*PI/180;
    }
}

void plannerLib::refreshGlobalPath(const nav_msgs::Path::ConstPtr& path)
{
    float distance = 0;     // Distance of the Path
    int cp = 0;             // Current Point

    // Clear the current Path
    originalPathX.clear();
    originalPathY.clear();

    // Clear the current RestList
    restPath.clear();

    while (cp < path->poses.size())
    {
        if (distance < PATH_LENGTH)
        {
            std::vector<float> pose;
            pose.push_back(path->poses.at(cp).pose.position.x); // 0
            pose.push_back(path->poses.at(cp).pose.position.y); // 1

            // Move x,y
            pose.push_back(pose.at(0)-globalCoords[0]);         // 2
            pose.push_back(pose.at(1)-globalCoords[1]);         // 3

            // Rotate
            pose.push_back(pose.at(2)*cos(globalCoords[2]) - pose.at(3)*sin(globalCoords[2]));  // 4
            pose.push_back(pose.at(2)*sin(globalCoords[2]) + pose.at(3)*cos(globalCoords[2]));  // 5

            // Save Value
            originalPathX.push_back(pose.at(4));
            originalPathY.push_back(pose.at(5));

            // Update Distance
            if (cp > 0)
                distance += sqrt(pow( (pose.at(0) - originalPathX.at(cp-1)), 2) +
                                 pow( (pose.at(1) - originalPathY.at(cp-1)), 2));

        } else {
            std::vector<float> appendedPoint;
            appendedPoint.push_back(path->poses.at(cp).pose.position.x);
            appendedPoint.push_back(path->poses.at(cp).pose.position.y);

            restPath.push_back(appendedPoint);
        }
    }

}

void plannerLib::handleNewCostmap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData data = msg->info;

    std::vector<int> costmap;
    int width = data.width;
    int height = data.height;
    float res = data.resolution;

    // Read in the costmap
    for (unsigned int x = 0; x < width; x++)
      for (unsigned int y = 0; y < height; y++)
          costmap.push_back(msg->data[x + width * y]);

    carWidth = CAR_WIDTH / res;
    carLength = CAR_LENGTH / res;


    float bestcost;
    std::vector< std::vector<float> > bestPath;

    // The starting-Point is set to the middle of the costmap
    std::vector<float> startPoint(2);
    startPoint.push_back(width/2);//originalPathX.at(0);
    startPoint.push_back(height/2);//originalPathY.at(0);

    // Set the end-Point
    std::vector<float> endPoint(2);
    endPoint.push_back(originalPathX.at(originalPathX.size()-1));
    endPoint.push_back(originalPathY.at(originalPathY.size()-1));


    // Check the original Path
    std::vector<int> pointsX, pointsY;
    if (false) {    // If the globalPath always starts at the car Position
        createCalcPoints(originalPathX, originalPathY, pointsX, pointsY);
    } else {        // The middle of the costmap is the position of the car
        std::vector<float> artificialPathX, artificialPathY;
        createArtificialPath(startPoint, endPoint, originalPathX.size(),artificialPathX, artificialPathY);
        createCalcPoints(artificialPathX, artificialPathY, pointsX, pointsY);
        bestPath = convertToBestPath(artificialPathX, artificialPathY);
    }

    bestcost = calcCost(pointsX, pointsY, costmap, width, height);


    // If the cost of the Original Path is over the Threshold
    if (bestcost > THRESHOLD)
    {
        std::vector<float> mPoP(2);    // Middle Point of Path
        mPoP.push_back(startPoint.at(0) + (endPoint.at(0) - startPoint.at(0))/2);
        mPoP.push_back(startPoint.at(1) + (endPoint.at(1) - startPoint.at(1))/2);

        for (float alpha = -rad(MAX_ANGLE); alpha <= rad(MAX_ANGLE); alpha += RESOLUTION)
        {
            float beta;

            if (endPoint.at(0) == startPoint.at(0))
                beta = (PI/2) - alpha;
            else if (endPoint.at(0) < startPoint.at(0))
                beta =  atan((endPoint.at(1) - startPoint.at(1))/
                             (endPoint.at(0) - startPoint.at(0)))  - alpha;
            else
                beta =  atan((endPoint.at(1) - startPoint.at(1))/
                             (endPoint.at(0) - startPoint.at(0)))  + alpha + PI;

            float mp_length = sqrt(pow(mPoP.at(2)-startPoint.at(1),2) + pow(mPoP.at(1)-startPoint.at(0),2)) / sin((PI/2) - alpha);
            std::vector<float> cmp(2);                                      // Current Middel Point of new Path
            cmp.push_back(startPoint.at(0) + mp_length*sin((PI/2)-beta));
            cmp.push_back(startPoint.at(1) + mp_length*sin(beta));

            std::vector<float> alternativPathX, alternativPathY;

            for (float t = 0; t <= 1; t+=(1/(originalPathX.size()-1)))
            {
                float aX = getPt_bezier(originalPathX.at(0), cmp.at(0), t);
                float aY = getPt_bezier(originalPathY.at(0), cmp.at(1), t);
                float bX = getPt_bezier(cmp.at(0), originalPathX.at(originalPathX.size()-1), t);
                float bY = getPt_bezier(cmp.at(1), originalPathY.at(originalPathY.size()-1), t);

                alternativPathX.push_back( getPt_bezier(aX, bX, t) );
                alternativPathY.push_back( getPt_bezier(aY, bY, t) );
            }


            createCalcPoints(alternativPathX, alternativPathY, pointsX, pointsY);
            float currentCost = calcCost(pointsX, pointsY,costmap,width,height);

            if(currentCost < bestcost)
            {
                bestcost = currentCost;
                bestPath.clear();
                bestPath = convertToBestPath(alternativPathX, alternativPathY);
            }
        }
    }

    ownPath.poses.clear();

    // Convert Own Local Path to Global Path
    for (int i = 0; i < bestPath.size(); i++)
    {
        std::vector<double> op;
        op.push_back(bestPath.at(i).at(0));         // 0
        op.push_back(bestPath.at(i).at(1));         // 1

        // Rotate back
        op.push_back(op.at(0)*cos(-globalCoords[2]) - op.at(1)*sin(-globalCoords[2]));  // 2
        op.push_back(op.at(0)*sin(-globalCoords[2]) + op.at(1)*cos(-globalCoords[2]));  // 3

        // Move back
        op.push_back(op.at(2)+globalCoords[0]);     // 4
        op.push_back(op.at(3)+globalCoords[1]);     // 5

        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = op.at(4);
        ps.pose.position.y = op.at(5);
        ownPath.poses.push_back(ps);
    }

    // Append the rest of the old Path
    for (int i = 0; i < restPath.size(); i++)
    {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = restPath.at(i).at(0);
        ps.pose.position.y = restPath.at(i).at(1);
        ownPath.poses.push_back(ps);
    }

    path_Pub.publish(ownPath);
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

/*  CREATE ARTIFICIAL PATH
 *
 *  This function creates waypoints linear from a starting to an endpoint, with a given resolution
 *
 *                                                                                                                      */
void plannerLib::createArtificialPath(std::vector<float> startPoint, std::vector<float> endPoint, int steps, std::vector<float> &newPathX, std::vector<float> &newPathY)
{
    float diffX = endPoint.at(0) - startPoint.at(0);
    float diffY = endPoint.at(1) - startPoint.at(1);
    newPathX.clear();
    newPathY.clear();
    for (int i = 0; i <= steps; i++)
    {
        newPathX.push_back(startPoint.at(0)+i*(diffX/(steps-1)));
        newPathY.push_back(startPoint.at(1)+i*(diffY/(steps-1)));
    }
}


/*  CREATE CALC POINTS
 *
 *  This Function creats a List of points, which are needed to calculate the cost of the current Path
 *
 *                                                                                                                      */
void plannerLib::createCalcPoints(std::vector<float> coordinatsX, std::vector<float> coordinatsY, std::vector<int> &xPoints, std::vector<int> &yPoints)
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
            xPoints.push_back((int)floor(xPointsCar.at(i)+0.5f));
            yPoints.push_back((int)floor(yPointsCar.at(i)+0.5f));
        }
    }
}

/*  CALCULATE COST
 *
 *  This Function calculates the Costs to the Points out of CREATE CALC POINTS
 * .
 *                                                                                                                      */
float plannerLib::calcCost(std::vector<int> xPoints, std::vector<int> yPoints, std::vector<int> costMap, int width, int hight)
{
    // There must be an equal number of X and Y Coordinates
    if (xPoints.size() != yPoints.size()){
        ROS_ERROR("Wrong use of calcCost");
        return -1;
    }

    float cost = 0;
    for (int i = 1; i < xPoints.size(); i++)
    {
        // If the calculated point is outside the costmap
        if (xPoints.at(i) < 1 || yPoints.at(i) < 1 | xPoints.at(i) > width | yPoints.at(i) > hight)
            cost += COST_OUTSIDE;
        // If the cost at this point is undefined
        else if (costMap.at(getIndex(xPoints.at(i), yPoints.at(i), width, hight)) < 0)
            cost += COST_OUTSIDE;
        // ELSE get the cost out of the costmap
        else
            cost += costMap.at(getIndex(xPoints.at(i), yPoints.at(i), width, hight));
    }

    return cost;
}


std::vector< std::vector<float> > plannerLib::convertToBestPath(std::vector<float> xVals, std::vector<float> yVals)
{
    if (xVals.size() != yVals.size())
    {
        ROS_ERROR("Wrong use of convertToBestPath");
    }

    std::vector< std::vector<float> > result;
    for (int i = 0; i < xVals.size(); i++)
    {
        std::vector<float> point(2);
        point.push_back(xVals.at(i));
        point.push_back(yVals.at(i));
        result.push_back(point);
    }
    return result;
}


/*  Calculates the Index for Vector of Costmap to given x and y points
 *
 *                                                                                                                      */
int plannerLib::getIndex(int x, int y, int width, int hight)
{
    return (y-1)*hight + (x-1);
    //return (x-1)*width + (y-1);   // IF FLIPPED
}

float plannerLib::rad(float deg)
{
    return deg*PI/180;
}

int plannerLib::getPt_bezier(int n1, int n2, float perc)
{
    return n1 + ((n2-n1)*perc);
}




