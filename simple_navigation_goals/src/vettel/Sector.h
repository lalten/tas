#ifndef SECTOR_H
#define SECTOR_H

/*********************************
 * Author:  Konrad Vowinckel    *
 * Kennung: ga68jan             *
 * ******************************/

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>

class Sector
{
public:
    Sector(std::string id, float minX, float maxX, float minY, float maxY, float targetX, float targetY, float targetW);
    bool isInSector(geometry_msgs::Pose currentPose);
    geometry_msgs::Pose getTargetPose();
    std::string getID();

private:
    float m_minX;
    float m_maxX;
    float m_minY;
    float m_maxY;
    float m_targetX;
    float m_targetY;
    float m_targetW;

    std::string m_id;
};

#endif
