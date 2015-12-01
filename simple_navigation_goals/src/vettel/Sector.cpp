/*********************************
 * Author:  Konrad Vowinckel    *
 * Kennung: ga68jan             *
 * ******************************/

#include "Sector.h"

Sector::Sector(std::string id,
        float minX, float maxX, float minY, float maxY,
        float targetX, float targetY, float targetW)
{
    this->m_minX = minX;
    this->m_maxX = maxX;
    this->m_minY = minY;
    this->m_maxY = maxY;
    this->m_targetX = targetX;
    this->m_targetY = targetY;
    this->m_targetW = targetW;

    this->m_id = id;
}

bool Sector::isInSector(geometry_msgs::Pose currentPose)
{
    return (currentPose.position.x > m_minX && currentPose.position.x < m_maxX &&
            currentPose.position.y > m_minY && currentPose.position.y < m_maxY );
}

geometry_msgs::Pose Sector::getTargetPose()
{
    geometry_msgs::Pose newTarget;
    newTarget.position.x = m_targetX;
    newTarget.position.y = m_targetY;
    newTarget.position.z = 0.000;
    newTarget.orientation.x = 0.000;
    newTarget.orientation.y = 0.000;
    newTarget.orientation.z = 0;
    newTarget.orientation.w = m_targetW;
    return newTarget;
}

std::string Sector::getID(){
    return m_id;
}
