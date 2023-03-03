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

### Compontents Diagram
In the following temporal diagram is showed he communication between the nodes.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/erl2_comp.PNG)  
The initialization is handeled by the "cluedo_state_machine" node, that calls the appropriate services of the ARMOR server to make the ontology ready for the game,  
and calls the service server /generate_murder to generate a winning hypothesis and store it for the following comparisons.  
During the game, the "cluedo_state_machine" node asks for hints to the oracle calling the service /get_hint and it get an hypothesis. Such hypothesis will be uploaded on the ontology and the ARMOR sever will be asked to reason with the new information and retireve the classes "COMPLETE" and "INCONISTENT". If the hypothesis just uploaded is part of the class "COMPLETE" but not of the class "INCONISTENT", that means that it is a consistent hypothesis and can be queried to the Oracle. The "cluedo_state_machine" node does so by calling the service /verify_solution that will return 3 booleans for each element oof the hypothesis (person, weapone, place).
If all the booleans are true the hypothesys is correc and the game ends.

### State Machine
In this iteration the flow of the game is planned by ROSplan services. ROSplan uses a PDDL file that defines a planning domain where the robot must move between waypoints and an oracle to solve the mystery. The robot can collect hints at waypoints, verify that the collected hypothesis is consistent, and eventually visit an oracle to check if its hypothesis is correct. 
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_sm.PNG)
Every state is defines by a durative action in the PDDL domain. They are "leave_oracle", "collect_hint", "go_to_next_point", "complete_query", and "solution_query". 
- "leave_oracle" represents the robot leaving an oracle and moving to a waypoint.
- "collect_hint" represents the robot collecting a hint at a waypoint.
- "go_to_next_point" represents the robot moving from one waypoint to another.
- "complete_query" represents the robot quering the ontology for consistency after hints have been taken from all waypoints.
- "solution_query" represents the robot checking whether its hypothesis is correct by visiting the oracle and receiving the solution.


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
```
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-moveit
sudo apt-get install ros-$ROS_DISTRO-moveit-ros-visualization
```
It is also dependent on the package of the previous iteration of the assignment and can be found here:
```
  git clone https://github.com/RobReho/exproblab.git
```
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
