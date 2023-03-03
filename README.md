# Experimental robotic lab 2
Second assignment for the Experimental Robotic Lab course a.y.2021/2022

## Introduction
At this stage of the project the robot has implemented as a 2 wheeled mobile robot with a manipulator on top. The robot explores its environment and deduces hypotheses based on hints it finds at four different loacation in a simulated environment. The robot has been designed to move through the environment collecting hints at 4 different coordinates. If a consistent hypothesis can be deduced, the robot will go in the oracle location and express it in English. If the hypothesis is incorrect, the robot will continue exploring and finding new hints until a correct hypothesis is deduced.

## Software architecture

### Robot architecture
The robot has this structure...
It has a moveit configuration so that the arm can be controlled
### ROS nodes
The nodes in this package are:
- "cluedo_state_machine": handles the communication with the ARMOR server.
- "oracle": controls the hints generation and holds the solution of the game.


### ROS services
The nodes communicate with some customized services:
- "/ask_solution" of type Consistent
- "/checkconsistency" of type Consistent
- "/good_hint" of type Bool
- "/oracle_solution" of type Oracle
- "/oracle_hint" of type ErlOracle

### State Machine
In this iteration the flow of the game is planned by ROSplan services. ROSplan uses a PDDL file that defines a planning domain where the robot must move between waypoints and an oracle to solve the mystery. The robot can collect hints at waypoints, verify that the collected hypothesis is consistent, and eventually visit an oracle to check if its hypothesis is correct. 
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_sm.PNG)
Every state is defines by a durative action in the PDDL domain. They are "leave_oracle", "collect_hint", "go_to_next_point", "complete_query", and "solution_query". 
- "leave_oracle" represents the robot leaving an oracle and moving to a waypoint.
- "collect_hint" represents the robot collecting a hint at a waypoint.
- "go_to_next_point" represents the robot moving from one waypoint to another.
- "complete_query" represents the robot quering the ontology for consistency after hints have been taken from all waypoints.
- "solution_query" represents the robot checking whether its hypothesis is correct by visiting the oracle and receiving the solution.

### Compontents Diagram
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_comp.PNG)  
In this architecture is composed by six c++ nodes that are the action intefaces of ROSplan, one c++ node that implements the oracle behaviour that is generating hints and giving the solution ID. Other three nodes implement the necessary services for the game.
The "plan_update" node calls the ROSplan servers to dispach a plan and start the game. As long as the dispatch response is False the ROSplan services are called and the knowledge base updated by deleting the "hint_taken" and "hypothesis_consistent" predicates in order to restart with the initial conditions in case an action fails.  
The node "simulation" is the oracle of the game. It generates hints relative to 6 different IDs. One of the ID is associated with the winning hypothesis, while all the others generates random hints that can sometimes also be defective. The node also implements the server callback that gives back the winning ID.  

The node "myhint" handle the processing of the hints and the ARMOR requests. It subscribes to the topic /oracle_hint, and receives all the hints detected by the robot. The hint is examined and only the well formed hints are advertised to the topic /good_hint. The node is also the client for the sevice /oracle_solution as it asks for the winning ID and compares it with the consistent ID from the ontology. It also sincronizes the communication with the rosplane interfaces, returning booleans as result for the rosplan actions.  

The node "go_to_oracle" enables the robot to move to a specific location while facing a designated direction. Initially, the robot aligns itself with the desired direction and progresses towards the goal. After arriving at the target coordinates, the robot adjusts its orientation by rotating itself to the correct position. The angular and linear velocities are determined by the user_interface node and are transmitted via the /cmd_vel topic. If the action server's client cancels the goal, all velocities are set to zero and the action server is preempted. This action service is called by all the ROSplan nodes that require the robot to move between points, which are "LeaveOracle", "GoToNextPoint", and "GoToOracle".

The node FromHomeAction.cpp implements the action defined in the domain as move_from_home. This action implements the motion of the robot from the home position to a predefined waypoint. We can see the same behaviour from the nodes ToHomeAction.cpp (for the action go_home) and MoveAction.cpp ( for the action goto_waypoint). These three nodes implements the exact same behavior but they are associated to three different action in the domain since there is the need to recognize the home position with a predicate that needs to be set to true and false when the robot reaches the home position or moves from the home position. All three nodes call the action server go_to_point that is implemented in the node go_to_point.py.
The node go_to_point.py implements the motion of the robot as an action server; it receives a desired position and orientation and it sends the required velocities to the robot. The motion is divided in three phases:

ROSplan action interfaces:
- LeaveOracle.cpp: Rosplan action called when the planner dispatches the action "leave_oracle" it moves the robot to the position of the first waypoint by calling the action server "go_to_point".
- CollectHint.cpp: implements the behaviour for the robot when the planner dispatches the action "collect_hint". It moves the arm to catch the hints by calling the moveit sever.  
- GoToNextPoint.cpp: Rosplan action called when the planner dispatches the action (go_to_next_point). It moves the robot to the desired waypoint retrieved by the rosplan dispatch message by calling the action server "go_to_point".
- CompleteQuery.cpp: Rosplan action called when the planner dispatches the action (complete_query). The callback calls the server of /checkcomplete service to check if there are any ID that have collected at least three hints and check if they form a consistent hypothesis. The action returns succesfully if at least one hypothesis is consistent, it fails otherwise.
- GoToOracle.cpp: Rosplan action called when the planner dispatches the action (go_to_oracle). It moves the robot to the Oracle position 0.0 by calling the action server "go_to_point".
-  SolutionQuery.cpp: Rosplan action called when the planner dispatches the action (solution_query). It calls the service /ask_solution asking the node "myhint" to retrieve the winning ID from the node "simulation" and compare it to the consistent hypothesis saved in the array "cIDs". The action is succesful if the IDs are the same, it fails otherwise.


## Installation and Running
This project needs some external packages. You can install them in your ROS workspace:  
ARMOR
```
  git clone https://github.com/EmaroLab/armor.git
```
SMASH (for assignment 1 package)
```
  git clone https://github.com/ros/executive_smach.git
  git clone https://github.com/ros-visualization/executive_smach_visualization.git
```
MOVEIT (Warning! this package works for MoveIt 1.1.5)
To install version 1.1.5 follow the tutorial: ![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/moveit1.1.5_installation_tutorial.txt) 


To install the package clone the repository in your ROS workspace:
```
  git clone https://github.com/RobReho/exproblab2.git
```
then build your ROS workspace:
```
  catkin_make
```

To run the project launch gazebo:
```
  roslaunch erl2 gazebo_mvoveit.launch
```
In another tab, launch rosplan:
```
  roslaunch erl2 rosplan.launch 2>/dev/null
```
This last tab is also the interface where the status of the game can be followed.

## Demo

![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/State_machine.gif)

## Working hypothesis and environment.
### System features
The game is a revisited simulated Cluedo game, where the player is the robot implemented by the state machine, and the game is controlled by the Oracle. The oracle gnerates the hypothesis by choosing random elements in the people, weapons and places arrays. Randomly, it might generate and inconsistent hypothesis, meaning that it will be composed by 4 elements instead of 3.
The robot will get both consistent and inconsistent hipothesis and send bach only the consistent hypothesis to be compared with the solution. When an hypothesis is compared to the solution the hints that don't match are deiscarded from the hints arrays stored in the oracle node. As more hypothesis are compared it is more and more likely that the hypothesis proposed matches the solution.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/erl1_end.PNG)

### System limitations 
The game implemented has a very simple structure and does not use the any IDs associated with the hypothesis. The architecture of the game has a very different way to generate and handle hypothesis with respect to the following iterations. Nevertheless, the semplicity of the architecture makes it easy to adapt to futures implementations.
### Possible technical Improvements
Possible improvement are a system that generates hints in a similar way to what happens in the following iterations.

## Contacts
Roberta Reho: s5075214@studenti.unige.it
