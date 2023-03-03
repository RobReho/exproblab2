/** @ package erl2
* 
*  \file GoToNextPoint.cpp
*  \brief implements the (go_to_next_point) action
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
*  Rosplan action called when the planner dispatches the action (go_to_next_point). 
*  It moves the robot to the desired waypoint.
*/
#include "erl2/InterfaceAction.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <string> 

#define PI 3.14159

namespace KCL_rosplan 
{
	GoToNextPointInterface::GoToNextPointInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
	
/**
 * \brief: MoveInterface callback
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return 0
 * 
 * Rosplan callback for the action go_to_next_point
 */		
	bool GoToNextPointInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{


		// here the implementation of the action 
		// prints to the screen the path the robot should do
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		// initialization of the action server client
		actionlib::SimpleActionClient<erl2::PlanningAction> ac("/go_to_point", true);
		// initialize the variable to send to the action server
		erl2::PlanningGoal goal;
		ac.waitForServer();
		// if the second parameter from the plan is wp1
		if(msg->parameters[1].value == "wp1")
			{
				// set the goal coordinates
				goal.target_pose.pose.position.x = 2.5;
				goal.target_pose.pose.position.y = 0.0;
				goal.target_pose.pose.orientation.w = 0.0;
			}
		// if the second parameter from the plan is wp2
		else if (msg->parameters[1].value == "wp2")
			{
				// set the goal coordinates
				goal.target_pose.pose.position.x = 0.0;
				goal.target_pose.pose.position.y = 2.5;
				goal.target_pose.pose.orientation.w = PI/2;
			}
		// if the second parameter from the plan is wp3	
		else if (msg->parameters[1].value == "wp3")
			{
				// set the goal coordinates
				goal.target_pose.pose.position.x = -2.5;
				goal.target_pose.pose.position.y = 0.0;
				goal.target_pose.pose.orientation.w = PI;
			}
		// if the second parameter from the plan is wp4
		else if (msg->parameters[1].value == "wp4")
			{
				// set the goal coordinates
				goal.target_pose.pose.position.x = 0.0;
				goal.target_pose.pose.position.y = -2.5;
				goal.target_pose.pose.orientation.w = -PI/2;
			}
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
	ros::init(argc, argv, "go_to_next_point", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoToNextPointInterface my_aci(nh);
	my_aci.runActionInterface();
	

	return 0;
}
