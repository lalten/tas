#ifndef HELPER_H
#define HELPER_H

/*********************************
 * Author:  Konrad Vowinckel    *
 * Kennung: ga68jan             *
 * ******************************/

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "Sector.h"

class helper
{
public:
    helper();
    void setCurrentWaypoints(float x, float y, float w, float z);
    bool flag_newGoal;
    geometry_msgs::Pose getNewGoal();

private:
    std::vector<Sector> m_allSectors;

    geometry_msgs::Pose m_currentWaypoint;
    geometry_msgs::Pose m_goalPose;

    void updateSector();


};

#endif // JOY_LIB_H
