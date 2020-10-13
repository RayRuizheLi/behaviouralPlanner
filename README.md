# Behavioural Planner 
A finite-statemachine based behavioural planner module written in C++. This project was built in collaboration with Rowan Dempster during my 2020 Summer COOP. The exact responsibilities are highlighted in the [Features](#Features) section.

The behaviour planner system plans the set of high level driving actions/maneuvers, to safely achieve the driving mission under vaiours driving situations. Behaviour planner considers the rules of the road, static, and dynamic objects around the vehicle. The behavioural planner is part of the hierachical framework for an autonomous vehicle. For more information, on behavioural planner, I recommend checkingout [this research paper](https://www.ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf)  by Junqing Wei. The result of the Readme aims to provide a highlevel break down of my part of the project.
<div align="center">
  <img src="https://github.com/RayRuizheLi/behaviouralPlanner/blob/main/readmeResources/autonomousHierarchy.png" alt="autonomous vehicle hierarchy" title="hierarchy"/>
</div>

# Input
Routing Input: 
* A queue of global commands
<div align="center">
  <img src="https://github.com/RayRuizheLi/behaviouralPlanner/blob/main/readmeResources/Town05.jpg" alt="autonomous vehicle hierarchy" title="hierarchy"/>
</div>
For example, the route illustrated in the image above would contain 3 global commands. 

Perception Input: 
* Coordinates of traffic signs
* Type of traffic signs
* Coordinates of traffic lights
* Traffic light signals 
* Stopline
* Coordinates of pedestrian 
* Other

# Output
* Signal to follow lane
* Signal to lane change 
* Signal to turn 
* Signal to stop

# Core Features
* Driving Commands (following lane, stop, turn, lane change) (Ray) 
* React to pedestrian (Rowan)
* React to traffic light (Rowan)
* React to stop signs (Rowan) 

# How it works
This section focuses on the driving commands functionality that I worked on. It will not focus on how react to pedestrian nor other features. 

### Finite-state Machine Architecture
The behavioural module uses a finite-state machine architecture. Each external and internal component is represented by a finite state machine. For example, each pedestrian, each traffic light, is represented by a finte-state machine. 
<div align="center">
  <img src="https://github.com/RayRuizheLi/behaviouralPlanner/blob/main/readmeResources/abstractStateMachine.png" alt="abstract state-machine code" title="abstract state-machine"/>
</div>
Each finite-state machine holds the current state and a transition graph. When a trigger event occurs, the finite-state machine transitions into the next state based on said transition graph. Since the finite statemachine is deterministic, each trigger only has one single resulting state. Here, two stat-machines will be explained in details: the ego state machine and the global command state machine. Together, these two state machine allows the behavioural planner to give driving commands.

### Ego State Machine 
Ego state machine holds the state of the car. The following is an illustration of its transition graph. Each arrow represents a trigger.

<div align="center">
  <img src="https://github.com/RayRuizheLi/behaviouralPlanner/blob/main/readmeResources/egoStateMachineTransitionGraph.png" alt="abstract state-machine code" title="abstract state-machine"/>
</div>

The ego state machine functions together with the global command state machine to drive the car. The following is an illustration of its transition graph. Again, each arrow represents a trigger. 

<div align="center">
  <img src="https://github.com/RayRuizheLi/behaviouralPlanner/blob/main/readmeResources/globalCommandStateMachine.png" alt="abstract state-machine code" title="abstract state-machine"/>
</div>

Here is an example of how the two state-machines work together. The upper stream routing module sends the global commands [(right_turn, x1, y1), (left_turn, x2, y2), (stop, x3, y3)]. 

* Gobal commands are sent

Both ego and global command starts at the waiting_for_command stage. Once the behavioural planner receives the set of global commands, the global command state machine pops the first command and transitions to the turning_right state. Now that ego state machine knows that there are commands, it transitions to the following_lane stage. Assuming we started at the right lane, we do not need to change lange. Therefore we are still at following_lane stage. 

* Reaches coordinates x1, y1

Global command state machine transitions to done stage since we arrived at x1, y1. Global state machine pops the command. Since the current cummand is now left-turn, the global state machine transitions to turning_left stage. Ego state machine sees that the global state machine is in turning_left stage. Since it is currently on the right lane, the car needs to change to the left lane to prepare for the left turn. Therefore the ego state machine transitions to changing lane left. Once the ego vehicle has stabilized, it transitions back to following lane. 

* Reaches coordinates x2, y2

Global command state machine transitions to done stage since we arrived at x2, y 2. Global state machine pops the command. Since the current command is stopm . Here, the behavioural planner creates a stop line. at x3, y3. Assuming x3, y3 is on the same lane, ego vehicle stays on following_lane stage. 

* Reaches coordinates x3, y3 

Global command state machine transitions to done stage since we arrived at x3, y3. Global state machine pops the command. Since no command is in the queue, the global stat machine transitions to the waiting_for_command stage. Since the ego vehicle reaches the stop line, it transitions to the stopping stage. Since the global state machine is in waiting_for_command stage, ego also transitions to the waiting_for_command stage. 

Here is a [video illustrating the process described above](https://drive.google.com/file/d/1xlQ9AcMrV05qT5aWn8n0a3e9FXjuKt3q/view?usp=sharing).

# What is Next 
* Increase the amount features such as other interacting with other vehicles.
* Increase the amount of states needed. 
* Maintaining the architecture as we encounter more scenarios.

