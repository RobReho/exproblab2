# Brief Introduction
This project implements the game of Cluedo played by a robot that has to explore an environment searching for hints. The robot should move in a single room looking for hints that are placed in known locations. When the robot is ready to make an hypothesis it should return to the center of the room and ask the oracle if the hypothesis is correct. The logic behind the behaviour of the robot is based on a planning problem and not a state machine.
# Software Architecture
![state diagram for the program](https://github.com/ZoeBetta/exprob_assignment2/blob/main/documentation/images/Architecture_Exprob_2.jpg)
This architecture is composed of ten nodes. Six of them implements the action related to the planning problem. There is one node for each action to be implemented.  
The node that starts the planning server and generates the problem is the planproblem.py node that controls and implements all the server needed to implement a planning problem. It sends the requests to the ROSPlan server and receives the feedbacks, based on which it then updates the knowledge_base if a replan is needed. In particular if one action fails ( only the check_complete action and the check_hypothesis action can fail) the node deletes from the knowledge base at least one hint_taken predicate and also sets to false the predicate hypothesis_complete in order to look for more hints.  
The node FromHomeAction.cpp implements the action defined in the domain as move_from_home. This action implements the motion of the robot from the home position to a predefined waypoint. We can see the same behaviour from the nodes ToHomeAction.cpp (for the action go_home) and MoveAction.cpp ( for the action goto_waypoint). These three nodes implements the exact same behavior but they are associated to three different action in the domain since there is the need to recognize the home position with a predicate that needs to be set to true and false when the robot reaches the home position or moves from the home position. All three nodes call the action server go_to_point that is implemented in the node go_to_point.py.  
The node go_to_point.py implements the motion of the robot as an action server; it receives a desired position and orientation and it sends the required velocities to the robot. The motion is divided in three phases: 
1. the robot aligns itself in the direction of the goal
2. the robot moves in a straight line towards the goal
3. the robot rotates to obtain the desired orientation  

The node Hint.cpp implements the action take_hint. When it is dispached the robot waits for an hint to be received.  We know that the hints can be found at the waypoint either in a low position or a high position. If, when the robot reaches the waypoint, no hint is received then the height of the end effector is the wrong one and the robot must move the arm to reach the other level. When the hint is retrieved it is sent as a request to the server /hint that is implemented in the node hint.py and has the goal of checking if the hint is correctly formulated and saving it in the ontology.  
The node CompleteAction.cpp implements the action check_complete. This node calls the server /checkcomplete that is implemented in the node hint.py. It sends a request and receives a boolean that is true if there is at least one hypothesis that is complete and not consistent and false if no hypothesis is both complete and consistent.  
The node CorrectAction.cpp implements the check_hypothesis action. The node sends a request to the server /correcthypothesis that is implemented in the node hypothesis.py. The server returns true if the inquired hypothesis is the correct one and false otherwise.  
The node hypothesis.py implements the server /correcthypothesis. It subscribes to the topic /complete where it receives the list of ID of complete hypothesis. From the list it then picks one ID that has not been checked yet and it makes a request to the server /oracle_solution to retrieve the winning ID, then it compares the two IDs. If the two IDs are the same, meaning the hypothesis is correct, the node makes a request to the server /results passing as argument the winning ID to retrieve the fields of the hypotheis (who,what, where) and printing them on the screen.  
The node hint.py as previously stated implements the three servers /hint, /checkcomplete, /results. All of these servers are based on the knowledge in the ontology. The server /hint checks if the hint is correctly formed, if all fields are filled and if what is written in the fields is meaningful. In case everything is correct the hint is saved in the ontology. The server /checkcomplete retrieves from the ontology all of the hypothesis, identified by a unique ID, that are complete and all the hypothesis that are not consistent. It then checks if there is at least one hypothesis that is in the complete list but not in the inconsistent list. All of the hypothesis that are both complete and consistent are then published on the topic /complete, if at least one hypothesis has been retrieved the server returns true, false otherwise. The server /results receives an ID and has to retrieve from the ontology all of the fields of that specific hypothesis; the fields are then returned as response of the server.  
The node simulation.cpp implements the oracle, and so it implements the hint generation algorithm but also the server that returns the correct hypothesis. This node publishes the hint on the topic /oracle_hint whenever the cluedo link reaches some points in the room. The hint that is sent can be malformed or empty.

# Installation
In order to be able to successfully run this code some packages are needed: the armor server is needed in order to work with the ontology. The ontology is needed to save and reason on the hint retrieved during the game. The armor server to work properly needs to follow the installation procedure available at the following link: https://github.com/EmaroLab/armor. In particular it is necessary to run the following command ./gradlew deployApp in the armor folder. It is also suggested to check if the cluedo_ontology file is in the path specifid in the hinty.py file at line 279.  
The ROSPlan package is needed in order to run the planning server that given a domain file, a problem file is able to plan the action to reach the goal, and in case some action fails to replan. Also in this case it is suggested to check that the files are in the correct path specified in the file mylaunch.launch at lines 87 for the domain and 88 for the problem.   
In order to obtain them I suggest downloading the docker that can be obtained at the following link: https://hub.docker.com/r/carms84/exproblab  
Once downloaded the docker, or the manually created the workspace in order to run the package it is suggested to clone the exprob_assignment2 github repository in the path /ros_ws/src. From the repository it is needed to move the file Robot.urdf from inside the package to outside in the path: /ros_ws/src. Also the file InterfaceAction.h that is inside the include folder must be moved  to the path /ros_ws/devel/include/exprob_assignment2.  
  
To build the package run:  
catkin_make --only-pkg-with-deps exprob_assignment2

# Running Code
to run the code in a terminal run:

roslaunch exprob_assignment2 mylaunch.launch random:= true

random is a ros parameter needed to customize the behaviour of the robot. if random is set to true then once a replan is needed only one waypoint is set as not visited so the robot will explore only that waypoint before looking if there is a complete hypothesis. 
If random is set to false then everytime there is a replan all the waypoints are set as not visited and the robot will have to go to every waypoint before checking if there is at least one hypothesis that is both complete and consistent.

# How the code runs
The robot starts in the middle of the room, the home position, from there it starts moving looking for hints in predetermined location in the room. For each location the hint can be at a high position or low position as can be seen from the following picture.  
![room with marker hints](https://github.com/ZoeBetta/exprob_assignment2/blob/main/documentation/images/room.JPG)
When the robot reaches a location it first waits to see if the hint is received without moving the arm, this is the case in which, for example, the arm is low and also the hint is in the low position; if the hint is retrieved the move action is completed and the robot starts the take hint action. In order to see if an hint has been received the node that implements the moving action subscribes to the topic hint. If instead no hint is retrieved in the prevoious postion of the arm the robot moves the arm to the other position: like in the following video.

https://user-images.githubusercontent.com/77151364/175808805-849480b6-a4e9-49be-bca1-88ad02522374.mp4

This is the behavior for retrieving the hints, once all of the locations have been visited ( so the robot has retrieved four hints) it checks if there is at least one complete and consistent hypothesis. If there is not any complete and consistent hypothesis the action fails and the robot starts looking for new hints with two different behaviors depending on the random parameter.  
If instead there is at least one complete and consistent hypothesis the robot, if not already in the home position, should move there and check if one of the complete and consistent hypothesis is the winning one. If it is not the winning one the action fails and the robot replans and starts looking for new hints. If instead the hypothesis is the correct one the node retrieves the fields of the hypothesis of the winning ID and prints the following message on the screen. After that it stops and the user has to manually stop the software by pressing CTR+C

![what happens at the end](https://github.com/ZoeBetta/exprob_assignment2/blob/main/documentation/images/finished.JPG)

The hints are formed of three fields: who, what, where.  
The possible values that the person can have are: 
* missScarlett
* colonelMustard
* mrsWhite
* mrGreen
* mrsPeacock
* profPlum

The possible values that the weapon can have are:
* candlestick
* dagger
* leadPipe
* revolver
* rope
* spanner

The possible values that the location can have are:
* conservatory
* lounge
* kitchen
* library
* hall
* study
* bathroom
* diningRoom
* billiardRoom

# Working Hypothesis
In order to implement this game I started from some hypothesis on the nature of the game and the actual capabilities of the robot. The main goal while deciding how to develop this application has been to develop a flexible architecture that would allow for easy integration and improvements. For this reason there are a lot of nodes, one for each action. In this way if we want to modify the domain file by removing or adding an action it will be a matter of only changing one file.  

## System's Features
This system implements a planning algorithm. This allows for a more reactive architecture that is able to adapt to the possibility of failure. In the case any action fails the system deletes the previous plan and depending on the current state of the robot, what the predicates are, it replans and searches for a new plan. In this implementation only two actions can fail: check_complete action and check_hypothesis action. The choice of not making the other two actions able to fail will be explained in the next paragraphs.  
Flexibility is also increased with the ros parameter random that is set when the code is run. This parameter regulates the modality of the replan. When a replan is needed at least one location is set as not visited in order for the robot to look for more hints. If the random param is set to true only one location is set as not visited, the location will be chosen randomly between all the locations minus the one the robot has just been to, this is due to the fact that the robot can't retrieve two hints from the same location without visiting another one. If the random param is set to false then all the locations are set as not visited and the robot will visit all of them before checking if an hypothesis is complete.  
Having the random param true will guarantee a faster response in the end when the robot already has gained a lot of hints and it has more probability that just one hint will allow it to find the correct hypothesis but it will slow it down in the beginning when with high probability one hint is not enough to find the correct hypothesis. The random param set to false works in the opposite way. 
## System's Limitations
This system doesn't implement the possibility for the actions related to moving in the room and the action take_hint to fail. This could create problems in some applications. The moving actions can't fail since the robot moves in a room without any obstacle and the goal position are set to reachable points.  
For what concerns the take_hint action this is more problematic since it can fail by retrieving an hint that is not correctly formed. I decided to avoid for the possibility of it to fail since there might be problems with the replanning, the robot can't keep searching the hint in the same location after retrieving a malformed hint. On the other side this allows the robot to search if a complete and consistent hypothesis is available even if it is impossible, for example the robot does a turn on all the locations and retrieves four malformed hints. 
## Possible technical improvements
The architecture could be improved by allowing the hint action to fail and programming a recovery plan for that instance. This could be implemented by having a predicate stating the last location of the robot and having a constraint for taking a hint in a location that is not the same as the one where the last hint was searched.  
Another behaviour that was observed is the fact that, even if it is not required, the robot decides to first move to the home position before looking if an hypothesis is complete. This is probably due to the search algorithm chosen and the predicates used for the domain. This problem slows down the resolution of the game since the robot moves to the home position even if there is not any complete hypothesis to be made, a possible solution could be having the go_home action that as a preconditio must have the predicate (hypothsis_complete) set as true
## Authors
Zoe Betta  
s5063114@studenti.unige.it  
zoe.betta@gmail.com
