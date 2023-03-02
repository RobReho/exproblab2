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
*   /checkcomplete
*    
*
*  Description: <BR>
*  Rosplan action called when the planner dispatches the action (complete_query). 
*  The callback calls the server of /checkcomplete 
*  service
*/

#include "erl2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/Consistent.h>

ros::ServiceClient cons_client;

namespace KCL_rosplan 
{
	CompleteQueryInterface::CompleteQueryInterface(ros::NodeHandle &nh)
		{
			// here the initialization
		}

/**
 * \brief: CompleteQueryInterface callback
 * \param msg : variables from the plan dispatcher
 * 
 * \return 0
 * 
 * Rosplan callback for the action complete_action
 * sends a request to service server /checkcomplete
 */
	bool CompleteQueryInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		
		//initialize the variable to send as a request to the server
		erl2::Consistent data;
		// set the start to true
		std::cout<<"Calling consistency service "<<std::endl;
		data.request.req =true;
		// call the service Client
		cons_client.call(data);
		// read the response to the server
		// if the response is true
		std::cout<<"Consistency service: "<<data.response.res<<std::endl;
		if (data.response.res==true)
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

/**
 * \brief: main function
 * \param : None
 * 
 * \return 0
 * 
 * Main function, it initializes the node and the interface to 
 * associate the action of Rosplan to the actual implementation.
 */
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "complete_query", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::CompleteQueryInterface my_aci(nh);
	my_aci.runActionInterface();
	cons_client = nh.serviceClient<erl2::Consistent>("/checkconsistency");
	return 0;
}
