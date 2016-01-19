/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("FB: [X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);

}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation


    /*
    // READ IN WAYPOINTS
    string line;
    string info = "NumbWayPoints:";
    string pos = "Pos:";
    string orient = "Orient:";
    ifstream myfile ("TAS_GROUP_06-Waypoints.txt");
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            if (line)
            cout << line << '\n';
        }
        myfile.close();
    }*/

//    geometry_msgs::Pose waypoint1;
//    waypoint1.position.x = 10.50;
//    waypoint1.position.y = 11.0;
//    waypoint1.position.z = 0.000;
//    waypoint1.orientation.x = 0.000;
//    waypoint1.orientation.y = 0.000;
//    waypoint1.orientation.z = 0;
//    waypoint1.orientation.w = 1;


    // 1er Start-Punkt
    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 11.1;
    waypoint1.position.y = 18.0;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.76;
    waypoint1.orientation.w = 0.64;
    waypoints.push_back(waypoint1);

    // Nach ersten Ecke
    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 12.7;
    waypoint2.position.y = 6.1;
    waypoint2.position.z = 0.000;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = -0.05;
    waypoint2.orientation.w = 0.1;
    waypoints.push_back(waypoint2);

    // Vor der kritischen Ecke
    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 21.86;
    waypoint3.position.y = 5.4;
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = -0.03;
    waypoint3.orientation.w = 1.0;
    waypoints.push_back(waypoint3);

    // Nach der kritischen Ecke
    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 23.6;
    waypoint4.position.y = 7.67;
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = 0.71;
    waypoint4.orientation.w = 0.7;
    waypoints.push_back(waypoint4);

    // Nach der kritischen Ecke
    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 21.1;
    waypoint5.position.y = 19.6;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = 1.0;
    waypoint5.orientation.w = 0.03;
    waypoints.push_back(waypoint5);



    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates


    //ROS_INFO("Set Goal to: [X]:%f [Y]:%f [W]: %f", waypoint1.position.x, waypoint1.position.y, waypoint1.orientation.x);

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }


    return 0;
}
