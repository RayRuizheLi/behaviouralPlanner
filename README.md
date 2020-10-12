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

# Features
# How it works
# Tech/Frameworks used 
# Features 
# Code Example
# Tests
# Credits
# Research and publications
