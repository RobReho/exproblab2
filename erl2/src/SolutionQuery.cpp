/** @ package erl2
* 
*  \file SolutionQuery.cpp
*  \brief implements the (solution_query) action
*
*  \author Roberta Reho
*  \version 1.0
*  \date 22/02/2023
*  \details

*   Client Services: <BR>
*   /correcthypothesis
*    

*  Description: <BR>
*  Rosplan action called when the planner dispatches the action (solution_query). 
*
*/

#include "erl2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/Consistent.h>

ros::ServiceClient sol_client;

namespace KCL_rosplan 
{
	SolutionQueryInterface::SolutionQueryInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
/**
 * \brief: SolutionQueryInterface callback
 * \param msg : variables from the plan dispatcher
 * 
 * \return 0
 * 
 * Rosplan callback for the action solution_query
 */
	bool SolutionQueryInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action
		 
		 // Call solution service
		erl2::Consistent data;
		std::cout<<"Calling solution service "<<std::endl;
		data.request.req =true;
		// call the service Client
		sol_client.call(data);
		// read the response to the server
		// if the response is true
		std::cout<<"Solution service: "<<data.response.res<<std::endl;
		// if the response is true
		if (data.response.res==true)
			{
				//print that the action was successful and return true to the planning server
				ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
				return true;
			}
		// if the response is false
		else 
			{
				// print that the action was not successfull and return false
				ROS_INFO("Action (%s) performed: wrong!", msg->name.c_str());
				return false;
			}
	}
}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "solution_query", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::SolutionQueryInterface my_aci(nh);
	my_aci.runActionInterface();
	// service client to ask for the solution ID
	sol_client = nh.serviceClient<erl2::Consistent>("/ask_solution");
	return 0;
}
