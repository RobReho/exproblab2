/** @ package erl2
* 
*  \file LeaveOracle.cpp
*  \brief implements the (leave_oracle) action
*
*  \author Roberta Reho
*  \version 1.0
*  \date 22/02/2023
*  \details

*  Action Services: <BR>
*    go_to_point
*
*  Description: <BR>
*  Rosplan action called when the planner dispatches the action (leave_oracle). 
*  It moves the robot to the position of the first waypoint.
*/

#include "erl2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

#define PI 3.14159

namespace KCL_rosplan {
	LeaveOracleInterface::LeaveOracleInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
	
/**
 * \brief: LeaveOracleInterface callback
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return 0
 * 
* Rosplan callback for the action leave_oracle
*  It moves the robot to the position of the first waypoint.
 */	
	bool LeaveOracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		// prints to the screen the path the robot should do
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		// initialization of the action server client
		actionlib::SimpleActionClient<erl2::PlanningAction> ac("/go_to_point", true);
		// initialize the variable to send to the action server
		erl2::PlanningGoal goal;
		ac.waitForServer();
		// return to waypoint 1 after oracle
		goal.target_pose.pose.position.x = 2.3;
	    goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		
		// send the goal to the action server
		ac.sendGoal(goal);
		// wait for the result from the action server
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

/**
 * \brief: main function
 * \param : None
 * 
 * \return 0
 * 
 * This is the main function, it initializes the node and the interface to 
 * associate the action of the ros plan to the actual implementation.
 */
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "leave_oracle", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::LeaveOracleInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
