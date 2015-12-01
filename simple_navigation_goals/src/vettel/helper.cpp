/*********************************
 * Author:  Konrad Vowinckel    *
 * Kennung: ga68jan             *
 * ******************************/

#include "helper.h"

helper::helper()
{
    flag_newGoal = false;

    /* SETUP SECTIONS */
    Sector s1("SECTOR 1",0.00,0.00,0.00,0.00,0.00,0.00,0.00);
    m_allSectors.push_back(s1);


}

helper::setCurrentWaypoints(float x, float y, float w, float z)
{
    m_currentWaypoint.position.x = x;
    m_currentWaypoint.position.y = y;
    m_currentWaypoint.orientation.w = w;
    m_currentWaypoint.orientation.z = z;
    updateSector();
}

void helper::updateSector()
{
    for (int i = 0; i < m_allSectors.size(); i++)
    {
        if (m_allSectors.at(i).isInSector(m_currentWaypoint)) {
            if (m_allSectors.at(i).getTargetPose() != m_goalPose){
                flag_newGoal = true;
                m_goalPose = m_allSectors.at(i).getTargetPose();
            }
            break;
        }
    }
}

void helper::getNewGoal()
{
    flag_newGoal = false;
    return m_goalPose;
}

