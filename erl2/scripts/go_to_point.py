#! /usr/bin/env python

## @package erl2
#
#  \file go_to_point.py
#  \brief This file implements the behaviour that allows the robot to reach a goal position.
#
#  \author Roberta Reho
#  \version 1.0
#  \date 03/03/2023
#  \details
#  
#  Subscribes to: <BR>
#	 /odom
#
#  Publishes to: <BR>
#	 /cmd_vel 
#
#  Services: <BR>
#    None
#
#  Action Services: <BR>
#    /go_to_point
#
#  Description: <BR>
#    This component enables the robot to move to a specific location while facing a designated direction. 
#    Initially, the robot aligns itself with the desired direction and progresses towards the goal. 
#    After arriving at the target coordinates, the robot adjusts its orientation by rotating itself to the correct position. 
#    The angular and linear velocities are determined by the user_interface node and are transmitted via the /cmd_vel topic. 
#    If the action server's client cancels the goal, all velocities are set to zero and the action server is preempted.


import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import erl2.msg
from std_msgs.msg import String, Float64

# robot state variables
position_ = Point()
pose_=Pose()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

desired_position_= Point()
desired_position_.z=0
success = False
# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6
Vel=Twist()

#action server
act_s=None

##
#	\brief Retrieves odometry data from the topic /odom
#	\param msg: the data received on the topic /odom
#	\return : None
# 	
#	The purpose of this function is to store the information received from the /odom topic 
#   subscriber into a global variable named "position," which contains details about the 
#   robot's current location. The function converts the orientation data from quaternion 
#   angles to Euler angles, and extracts the third element of the vector, which represents the yaw angle. 
#   This yaw angle is then stored in another global variable called "yaw_".
def clbk_odom(msg):
    #This is called when new data are available on /odom
    global position_
    global pose_
    global yaw_
	
    # save the current position
    position_ = msg.pose.pose.position

    # save the current orientation and yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
#   \brief Change state
#	\param state: the state of the robot
#	\return : None
# 	
#	This function modifies the global variable state_ to the provided state parameter, 
#   changing the current state of the program.
#   It prints a message to the console to inform the user of the new state.
def change_state(state):
    #Changes the state to a new one
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#	\brief Angles noramlization
#	\param angle: the angle to normalize
#	\return : angle, the normalized angle
# 	
#	This function normalizes the angle received as input
def normalize_angle(angle):
    #This makes calculations on angles
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#	\brief Robot yaw fix
#	\param des_pos: the desired x and y coordinates
#	\return : None
# 	
#	This function calculates the desired orientation and angular velocity 
#    required to reach a target point represented by x,y coordinates. 
#    It also changes the program's state to "go straight" if the orientation 
#   error falls below a certain threshold.
def fix_yaw(des_pos):
    #This orients the robot in direction of the goal
    global yaw_, pub, yaw_precision_2_, state_,Vel
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = normalize_angle(des_yaw - yaw_)
    
    # depending on the error the value of the angular velocity is calculated
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

##
#	\brief Go ahead
#	\param des_pos: the desired x and y coordinates
#	\return : None
# 	
#	The purpose of this function is to compute the desired orientation and the distance, 
#   both linear and angular, to reach a specified point represented by x,y coordinates. 
#   After determining the required velocities, the function sets the linear velocity and 
#   calculates the angular velocity based on the error in direction. Additionally, the function 
#   changes the program's state to the "fix final orientation" behavior if the distance between 
#   the goal falls below a certain threshold.
def go_straight_ahead(des_pos):
    #This makes the robot go in a straight line towards the goal
    global yaw_, pub, yaw_precision_, state_,Vel
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = des_yaw - yaw_
    err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
    err_yaw = normalize_angle(des_yaw - yaw_)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
#	\brief Robot final yaw fix
#	\param des_yaw: the desired orientation
#	\return : None
# 	
#	The purpose of this function is to calculate the error between the current 
#   and desired orientation and set the angular velocity accordingly. 
#   If the error falls below a specific threshold, the function changes
#   the program's state to "done."
def fix_final_yaw(des_yaw):
    global Vel
    # It orients the robot in the final desired orientation 
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
     
##
#	\brief Stop robot
#	\param : None
#	\return : None
# 	
#	This function puts to zero all the velocities, angular and linear, and sets
#	the goal as succeeded.  
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()

##
#	\brief Server behaviour
#	\param goal: the desired position and orientation to obtain
#	\return : None
# 	
#	This function is invoked upon receiving a request to the server. 
#   It initializes all required global variables and enters a while loop. 
#   Within the loop, the function continuously checks if the goal is preempted. 
#   If the goal is preempted, the function sets all velocities to zero and marks 
#   the goal as preempted. However, if the action server is not preempted, 
#   the function determines the current state and invokes the corresponding function.
def go_to_point(goal):
    print('request received')
    #Implements the logic to go to the goal
    global state_, desired_position_, act_s, success
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.orientation.w
    change_state(0)
    while True:
        # if the action is preempted
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            # I set the velocity to zero
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            # I set the goal as preempted
            act_s.set_preempted()
            success=False 
            break
        # orient the robot towards the goal    
        elif state_ == 0:
            fix_yaw(desired_position_)
        # go straight    
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        # orients in the final orientation    
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        # set the goal as achieved    
        elif state_ == 3:
            done()
            break
    return True


def main():
    global pub_, active_, act_s
    #initialize the goal
    rospy.init_node('go_to_point')
    #initialize the publisher for the velocity
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #initialize the subscriber on odometry
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    #initialize the action server
    act_s = actionlib.SimpleActionServer('/go_to_point', erl2.msg.PlanningAction, go_to_point, auto_start=False)
    act_s.start()
    rospy.spin()

if __name__ == '__main__':
    main()
