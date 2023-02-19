#!/usr/bin/env python
"""
In this script, we first import the necessary ROS messages and services from the rosplan_dispatch_msgs package. We then define a callback function called action_feedback_callback that will be called every time the ROSPlan plan dispatcher publishes an action feedback message. In this callback function, we check if the action status is ActionDispatch.FAILED, which indicates that the action failed to execute. If this is the case, we log an error message and call the dispatch_plan function to dispatch the plan again.

The dispatch_plan function uses the ROSPlan planner interface service /rosplan_planner_interface/planning_server to dispatch the plan again. We use a ServiceProxy object to call this service and handle any exceptions that might occur.

Finally, we define a main function that initializes the ROS node, subscribes to the /rosplan_plan_dispatcher/action_feedback topic, and starts the ROS event loop with rospy.spin(). The if __name__ == '__main__' block at the end of the script ensures that the main function is only called if the script is executed directly, not if it is imported as a module.
"""
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_dispatch_msgs.srv import *
from std_srvs.srv import *

""" @brief Callback function to handle action feedback messages.
 @param msg The ActionDispatch message received from the '/rosplan_plan_dispatcher/action_feedback' topic.
 """
def action_feedback_callback(msg):
    if msg.status == 10:    # failed
        rospy.logerr('Action %s failed, dispatching plan again', msg.action_id)
        dispatch_plan()

def dispatch_plan():
    # problem interface service
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    
    # planning interface service
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    plan_service = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    
    # parsing interface service
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    
    # dispatch plan
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    
    try:
        problem_generation()
        plan_service()
        parsing()
        dispatcher()
        rospy.loginfo('Plan dispatched successfully')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to dispatch plan: %s', e)

def main():
    rospy.init_node('replan')

    rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback , action_feedback_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
    """
    msg     rosplan_dispatch_msgs/ActionFeedback
    
    topic   /rosplan_plan_dispatcher/action_feedback
                rosplan_dispatch_msgs/ActionFeedback

            /rosplan_plan_dispatcher/dispatch_plan_action/cancel
            /rosplan_plan_dispatcher/dispatch_plan_action/feedback
            /rosplan_plan_dispatcher/dispatch_plan_action/goal
            /rosplan_plan_dispatcher/dispatch_plan_action/result
            /rosplan_plan_dispatcher/dispatch_plan_action/status
            """
   
