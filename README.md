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
- "user_interface": subscribes to the other node's messages and prints the on the terminal. The node is not represented in the following diagrams for sake of semplicity.

### ROS services
The nodes communicate with some customized services:
- "/verify_solution" of type Compare
- '/generate_murder' of type Hypothesis
- '/get_hint' of type Hints

### Temporal Diagram
In the following temporal diagram is showed he communication between the nodes.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/erl1_temp.PNG)  
The initialization is handeled by the "cluedo_state_machine" node, that calls the appropriate services of the ARMOR server to make the ontology ready for the game,  
and calls the service server /generate_murder to generate a winning hypothesis and store it for the following comparisons.  
During the game, the "cluedo_state_machine" node asks for hints to the oracle calling the service /get_hint and it get an hypothesis. Such hypothesis will be uploaded on the ontology and the ARMOR sever will be asked to reason with the new information and retireve the classes "COMPLETE" and "INCONISTENT". If the hypothesis just uploaded is part of the class "COMPLETE" but not of the class "INCONISTENT", that means that it is a consistent hypothesis and can be queried to the Oracle. The "cluedo_state_machine" node does so by calling the service /verify_solution that will return 3 booleans for each element oof the hypothesis (person, weapone, place).
If all the booleans are true the hypothesys is correc and the game ends.

### State Machine
The different states implement with the Smach package are showed below.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/sm1.PNG)
The states are implemented in the "cluedo_state_machine" node.
- The INIT state Establish the communication with Armor server, loads OWL file, calls "generate murder" service to start the game, retrieves people, weapons and places list from the OWL 
- The EXPLORE state retrieves the list of available places, randomly choose one and simulates reaching the place by sleeping 1 second.
- THe MAKE HYPOTHESIS state asks the server to get a new hint, it loads it on the ontology and retrieves the classes "COMPLETE" and "INCONISTENT" to check for consistency. If the hypothesis is consistent the executed state is REACH ORACLE, otherwise it will go back to the state EXPLORE.
- The state REACH ORACLE just simulates reaching the oracle posistion by sleeping 1 second. The possibility that this state fails is implemented in the state machine, but never executed. The possibility is left for future implementations where an actual sction will be implemented. Onche the oracle is reached, the next executed state is DELIVER HYPOTHESIS.
- The DELIVER HYPOTHESIS state gets the person, weapon and place of the hypothesis and express it in natural language. The next state is HYPOTHESIS CHECK
- The state HYPOTHESIS CHECK calls the server "verify solution" to compare the hypothesis with the right one. If all the booleans returned are true the game ends, otherwise the hypothesis is wrong and the program executes the state EXPLORE.

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
MOVEIT
```
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
