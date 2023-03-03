/** @ package erl2
* 
*  \file GoToOracle.cpp
*  \brief implements the ( go_to_oracle) action
*
*  \author Roberta Reho
*  \version 1.0
*  \date 22/02/2023
*  \details
*
*  Action Services: <BR>
*    go_to_point
*
*  Description: <BR>
*  Rosplan action called when the planner dispatches the action (go_to_oracle). 
*  It moves the robot to the Oracle position 0.0.
*/
#include "erl2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

namespace KCL_rosplan 
{
	GoToOracleInterface::GoToOracleInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
/**
 * \brief: GoToOracleInterface callback
 * \param msg : variables from the plan dispatcher
 * 
 * \return 0
 * 
 * Rosplan callback for the action go_to_oracle. 
 */		
	bool GoToOracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		// prints to the screen the path the robot should do
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		// initialization of the action server client
		actionlib::SimpleActionClient<erl2::PlanningAction> ac("/go_to_point", true);
		// initialize the variable to send to the action server
		erl2::PlanningGoal goal;
		ac.waitForServer();
		// set the goal coordinates
		goal.target_pose.pose.position.x = 0.0;
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


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "go_to_oracle", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoToOracleInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
