/** @ package erl2
* 
*  \file CompleteQuery.cpp
*  \brief implements the (complete_query) action
*
*  \author Roberta Reho
*  \version 1.0
*  \date 22/02/2023
*  \details
*   
* 
*   Client Services: <BR>
*   /checkconsistency
*    
*
*  Description: <BR>
*  Rosplan action called when the planner dispatches the action (complete_query). 
*  The callback calls the server of /checkcomplete service
*/

#include "erl2/InterfaceAction.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/Consistent.h>
#include <string> 

ros::ServiceClient cons_client;

namespace KCL_rosplan 
{
	CompleteQueryInterface::CompleteQueryInterface(ros::NodeHandle &nh)
		{
			// here the initialization
		}

/**
 * \brief: CompleteQueryInterface callback
 * \param msg : plan dispatcher variables 
 * 
 * \return 0
 * 
 * Rosplan callback for the action complete_action
 * sends a request to service server /checkconsistency
 */
	bool CompleteQueryInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		//initialize the variable to send as a request to the server
		erl2::Consistent srv;
		// set the start to true
		std::cout<<"Calling consistency service: "<<std::endl;
		srv.request.req = true;
		// call the service Client
		cons_client.call(srv);
		// read the response to the server
		// if the response is true
		if (srv.response.res==true)
			{
				//print that the action was successful and return true to the planning server
				ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
				return true;
			}
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
	ros::init(argc, argv, "complete_query", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	cons_client = nh.serviceClient<erl2::Consistent>("/checkconsistency");
	KCL_rosplan::CompleteQueryInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}
