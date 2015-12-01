/*********************************
 * Author:  Konrad Vowinckel    *
 * Kennung: ga68jan             *
 * ******************************/

#include "helper.h"

helper::helper()
{
    flag_newGoal = false;

    /* SETUP SECTIONS */
    //          ID         minX     maxX    minY        maxY        targetX     targetY     targetW
    Sector s1("SECTOR 1",-11.0,     0.6,    -0.7,       1.7,       -10.0,       -1.7,       1.57);
    Sector s2("SECTOR 2", -11.0,    -9.0,   -13.3,      -0.7,       -8.5,       -12.8,      3.14);
    Sector s3("SECTOR 3", -9.0,     2.0,    -13.3,      -11.3,      1.0,        -10.0,      4.71);
    Sector s4("SECTOR 4", 0.6,      2.0,    -11.3,      1.7,        0.0,        0.0,        0.0);

    m_allSectors.push_back(s1);
    m_allSectors.push_back(s2);
    m_allSectors.push_back(s3);
    m_allSectors.push_back(s4);
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

