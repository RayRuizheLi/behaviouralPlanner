#pragma once

#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <exception>
#include <string>
#include <stack>
#include <deque>

#include "DynamicNode/parameter_config.h"
#include "common/environment.hpp"
#include "common/lane.hpp"
#include "common/point2d.hpp"
#include "common/polyline.hpp"
#include "common/road_data.hpp"
#include "common/ros_msgs.hpp"
#include "global_command.hpp"

#include "behavioral/HDM_interface.hpp"

namespace path_planning{

class global_reroute : public std::exception {
  public:
  global_reroute(Lane curr) : current(curr) {}
  global_reroute(VehicleState ego_pose) {
    current.push_back(ego_pose);
  }
  Lane current;
  virtual const char* what() const throw() {
   return "Reroute requested";
  }
};

class GlobalInterface {

  GlobalCommandQueue global_cmd_queue;
  LaneTopology topology;
  std::deque<Lane> path;
  VehicleState ego_pose;
  HDMInterface& hdm_interface;

  // main planning algos
  bool planPath();
  bool breadthFirstSearch();

  /* Returns a current and desired lane containing only the vehicle pose */
  std::pair<Lane, Lane> doNothingLanes();
  /*
   * Computes the current and desired lane based on the path
   * Output: {currentLane, desiredLane}
   */
  std::pair<Lane, Lane> getCurrentDesiredLane();

  public:
  GlobalInterface(HDMInterface& hdm_interface) : hdm_interface(hdm_interface) {}

  /* Main Interface:
   * Plans a lane level path using the lane topology and global commands
   * Output: {currentLane, desiredLane}
   */
  std::pair<Lane, Lane> planLaneRoute();

  // utility
  bool pastFirstPathSegment();
  double distanceToPath();
  double getPathLength();

  // To be replaced by carter's state machine
  bool nearIntersection();

  // Accessors
  Lane& queryLane(int lane_id);

  void setGlobalCommandQueue(GlobalCommandQueue q) { global_cmd_queue = std::move(q); };
  void setPose(VehicleState p){ego_pose = std::move(p);}
  GlobalCommandQueue& getGlobalCommandQueue() { return global_cmd_queue; };
  GlobalCommandQueue getGlobalCommandQueue() const { return global_cmd_queue; };
  VehicleState getPose(){return ego_pose;}
  std::deque<Lane>& getPath() {return path;}
  std::deque<Lane> getPath() const {return path;}
  LaneTopology& getTopology() {return topology;}
  LaneTopology getTopology() const {return topology;}

  // Debug

  void DEBUG_PRINT_PATH(){
    ROS_DEBUG("path:");
    for (auto& lane : path){
      ROS_DEBUG_STREAM("\t" << lane);
    }
  }
};

}
