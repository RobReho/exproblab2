#! /usr/bin/env python

## @package erl2
#
#  
import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
import erl2.msg

# global variables
prev=4

##
#	\brief This function initializes all the server used for the planning part
#	\param : 
#	\return : None
# 	
#	This function initializes all the servers and waits for all of them to be 
#   running before moving forward
def initialization():
    global problem_generation, planning, parsing, dispatch, update
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    print('all servers loaded')

##
#	\brief This function deletes the hint_taken parameter from the knowledge base for one waypoint
#	\param name: it is the waypoint I want to modify 
#	\return : None
# 	
#	This function calls the knowledge base server to delete the predicate (hint_taken)
#   for one waypoint, the one recognized by the parameter name
def update_waypoint(name):
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hint_taken'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', name))	
    result=update(req)
    
##
#	\brief This function deletes the (hypothesis_complete) fact
#	\param :
#	\return : None
# 	
#	This function calls the knowledge base server to delete the predicate
#   (hypothesis_complete)
def update_complete():
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hypothesis_complete'
    result=update(req)	

##
#	\brief This function updates the knowledge base
#	\param :
#	\return : None
# 	
#	This function looks at the parameter in the ros param server random.
#   if random is set to true it finds a random waypoint ( different from the previously
#   calculated one) and it proceeds to delete the (hint_taken) for that waypoint.
#   In case the random parameter is set to false it deletes the fact (hint_taken) for
#   all waypoints. After that it deletes the (hypothesis_complete) fact.
def know_update():
        
    update_waypoint('wp1')
    update_waypoint('wp2')
    update_waypoint('wp3')
    update_waypoint('wp4')
    # delete the (hypothesis_complete) fact
    update_complete()

##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, then it calculates a new plan
#   until the robot is able to fuòòy complete one. 
def main():
    global pub_, active_, act_s
    rospy.init_node('plan_update')
    initialization()
    success=False
    goal=False
    # until the feedback from the planner is not true
    while ( success== False or goal == False):
		# generate the problem
        response_pg=problem_generation()
        print('problem generates')
        time.sleep(1)
        # generate the plan
        response_pl=planning()
        print('plan generates')
        time.sleep(1)
        # parse the plan 
        response_pars=parsing()
        print('parse generates')
        time.sleep(1)
        # read the feedback
        response_dis=dispatch()
        print(response_dis.success)
        print(response_dis.goal_achieved)
        success= response_dis.success
        goal= response_dis.goal_achieved
        # update the knowledge base
        know_update()
        time.sleep(1)
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()
