#! /usr/bin/env python

## @package erl2
#
#  \file go_to_point.py
#  \brief This file implements the ROSplan update callback
#
#  \author Roberta Reho
#  \version 1.0
#  \date 03/03/2023
#  \details
#
#  Services: <BR>
#    /rosplan_problem_interface/problem_generation_server
#    /rosplan_planner_interface/planning_server
#    /rosplan_parsing_interface/parse_plan
#    /rosplan_plan_dispatcher/dispatch_plan
#    /rosplan_knowledge_base/update


import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time

# global variables
prev=4

##
#	\brief This function initializes all the necessary servers of ROSplan
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
#	\brief knowledge base update
#	\param :
#	\return : None
# 	
#	This function calls the knowledge base server to delete the predicate (hint_taken)
#   for all the waypoints, and does the same for the  predicate hypothesis_complete
def knowledge_update():
    wps = ['wp1','wp2','wp3','wp4']
    
    # delete 'hint_taken' predicate for all waypoints
    for wp in wps:
        req=KnowledgeUpdateServiceRequest()
        req.update_type=2
        req.knowledge.is_negative=False
        req.knowledge.knowledge_type=1
        req.knowledge.attribute_name= 'hint_taken'
        req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', wp))	
        update(req)
    
    # delete the 'hypothesis_complete' predicate 
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hypothesis_complete'
    update(req) 
    


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
        print('Generating problem')
        time.sleep(1)
        # generate the plan
        response_pl=planning()
        print('Generating plan ')
        time.sleep(1)
        # parse the plan 
        response_pars=parsing()
        print('Generating parse ')
        time.sleep(1)
        # read the feedback
        response_dis=dispatch()
        print(response_dis.success)
        print(response_dis.goal_achieved)
        success= response_dis.success
        goal= response_dis.goal_achieved
        # update the knowledge base
        knowledge_update()
        time.sleep(1)
    print ('SUCCESFUL PLAN')

if __name__ == '__main__':
    main()
