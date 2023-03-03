/** @ package erl2
* 
*  
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
	ros::init(argc, argv, "go_to_oracle", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoToOracleInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
