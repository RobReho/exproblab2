/** @ package erl2
* 
*  \file CollectHint.cpp
*  \brief implements the (collect_hint) action
*
*  \author Roberta Reho
*  \version 1.0
*  \date 22/02/2023
*  \details
*   
*  Subscribes to: <BR>
*	/oracle_hint
*
*   Client Services: <BR>
*   /hint
*    
*  Action Services: <BR>
*    go_to_point
*
*  Description: <BR>
*  Rosplan action called when the planner dispatches the action (collect_hint). 
*  
*/

#include "erl2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <erl2/ErlOracle.h>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <vector>

// global variables
int received=0;
bool good_hint= false;
int ID0, ID1, ID2, ID3, ID4, ID5;


// function declaration 
int moveArm(std::string targetPosition);    // default - high_reach - low_reach
int count = 0; // safety counter
void hintCallback( const std_msgs::Bool::ConstPtr& Hint);
ros::Publisher ui;  // User Interface publisher

namespace KCL_rosplan 
{
    CollectHintInterface::CollectHintInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
	
/**
 * \brief: CollectHintInterface callback
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return true
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action take_hint. It moves the arm to retrieve the hints.
 */	
    bool CollectHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
        // move arm in the high position
		moveArm("high_reach");

        while(!received){
            std::cout << "Moving to get the hint " << std::endl;
            // move arm 
            moveArm("high_reach");
            moveArm("low_reach"); 
            count++;
            if(count>=2){
                count =0;
                break;
            }
        }
        // hint received
        received = 0;
        return true;

    }
}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "collect_hint", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	// initialization of the node handle for the subscriber on the topic /oracle_hint
	ros::NodeHandle n;
	ros:: Subscriber hint = n.subscribe("/good_hint", 1000, hintCallback);
	ui = nh.advertise<std_msgs::String>( "/ui_output", 30 );
	KCL_rosplan::CollectHintInterface my_aci(nh);
	my_aci.runActionInterface();
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sleep(2.0);
	return 0;
}

/**
 * \brief: callback for the topic /good_hint
 * \param Hint : the message received on the /good_hint topic
 * 
 * \return None
 * 
 * The callback sets the variable "received" to 1 and prints whether the
 * recaived hint is valid or not 
 */
void hintCallback( const std_msgs::Bool::ConstPtr& Hint){

    received = 1;
    good_hint = Hint->data;
    if(!good_hint){
        std::cout<<"Bad hint received!"<<std::endl;
    }else{
        std::cout<<"Good hint received!"<<std::endl;
    }
}



/**
 * \brief: Move robot arm using Moveit api
 * \param targetPosition : String describing the desired pose of the robot arm
 * 
 * \return 0
 * 
 * This function gets in inputthe desired position of the robot arm
 * and sets all the features needed for the planning and execution of the movement.
 */
int moveArm(std::string targetPosition)
{
	moveit::planning_interface::MoveGroupInterface group("arm");
    group.setEndEffectorLink("cluedo_link");
    group.setPoseReferenceFrame("base_link");
    group.setPlannerId("RRTstar");
    group.setNumPlanningAttempts(10);
    group.setPlanningTime(10.0);
    group.allowReplanning(true);
    group.setGoalJointTolerance(0.0001);
    group.setGoalPositionTolerance(0.0001);
    group.setGoalOrientationTolerance(0.001);
    group.setNamedTarget(targetPosition);
    group.move();
	return 0;
}





