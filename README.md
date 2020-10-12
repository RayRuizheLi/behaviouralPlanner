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

Each finite-state machine holds the current state and a transition graph. When a trigger event occurs, the finite-state machine transitions into the next state based on said transition graph. Since the finite statemachine is deterministic, each trigger only has one single resulting state. Here, two stat-machines will be explained in details: the ego state machine and the global command state machine. Together, these two state machine allows the behavioural planner to give driving commands.


### Finite-state Machine Architecture 


* How does state machine work?
* How does triggers work? 
* How does transition graphs work? 
* Briefly go over Pedestrian and Traffic Light transitions 
* Focus on EGO statemachine and globl command state machine
* includ code snip bits 
# Link videos 
# Tests
* tested in carla. 
* pretend perception is mocked by Carla
* Routing is done in script 
# What Next
* Talk about maintaining scenarios and transitions 
* Talk about dynaic obstacles such as car. 
# Research and publications
* link publications. 
