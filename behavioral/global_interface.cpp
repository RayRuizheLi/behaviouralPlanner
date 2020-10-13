#include "global_interface.hpp"
#include <stdexcept>

namespace path_planning {

std::pair<Lane, Lane> GlobalInterface::planLaneRoute() {

  if (global_cmd_queue.empty()){
    ROS_WARN_THROTTLE(20, "BP: NO GLOBAL COMMAND");
    return doNothingLanes();
  }
  ROS_INFO_STREAM_ONCE("BP: FIRST COMMAND " << global_cmd_queue.front());

  // initialization and error handling.
  if (path.empty() || (distanceToPath() > Dynamic_Config.path_cutoff_distance && !nearIntersection())){
    ROS_INFO_THROTTLE(20, "BP: STARTUP OR RETRY");
    if (!planPath()){
      path.clear();
      ROS_ERROR_THROTTLE(20, "BP: REQUIRE REROUTE, PATH SIZE: %zu, DEVIATION: %f, CUTOFF: %f", path.size(), distanceToPath(), Dynamic_Config.path_cutoff_distance);
      // TODO: Slowdown but keep driving
      throw global_reroute(ego_pose);
    }
  }

  // Check if we've completed the current command
  if (!nearIntersection() && global_cmd_queue.front().isComplete(ego_pose, path[0].start_link_id)){
    ROS_INFO_STREAM("BP: DONE COMMAND\nNEXT COMMAND: " << global_cmd_queue.front());
    global_cmd_queue.pop();
    // re-plan
    if (global_cmd_queue.empty()){
      path.clear();
      return doNothingLanes();
    }
    if (!planPath()){
      path.clear();
      ROS_ERROR("BP: REQUIRE REROUTE, FAILED TO PLAN PATH");
      // TODO: Slow down but keep going
      throw global_reroute(ego_pose);
    }
  }

  // check if we've finished with the current link
  if (pastFirstPathSegment()){
    ROS_INFO("DONE SEGMENT");
    path.pop_front();
    DEBUG_PRINT_PATH();
    return planLaneRoute();
  }

  return getCurrentDesiredLane();
}

bool GlobalInterface::nearIntersection(){
  if (global_cmd_queue.front().cmd == GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN ||
      global_cmd_queue.front().cmd == GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN ||
      global_cmd_queue.front().cmd == GlobalCommand::INSTRUCTION_TYPE::STRAIGHT){
    return (ego_pose.distanceTo(global_cmd_queue.front().point) < Dynamic_Config.safe_intersection_distance);
  } else {
    return false;
  }
}

// Returns true if the vehicle pose has passed the first segment in `path`
bool GlobalInterface::pastFirstPathSegment(){
  if (path.size() == 0 || (path.size() == 1 && path[0].size() == 0)) {
    return false;
  }
  if (path.size() == 1) {
    return path[0].back().distanceTo(ego_pose) < Dynamic_Config.lane_cutoff_distance;
  }
  else {
    return path[1].distanceTo(ego_pose) < Dynamic_Config.lane_cutoff_distance;
  }
}

// returns the minimum distance to the path
double GlobalInterface::distanceToPath(){
  if (path.size() == 0) {
    return 0;
  }
  double min = INT_MAX;
  for (auto& lane : path){
    int dist = lane.distanceTo(ego_pose);
    if (min > dist){
      min = dist;
    }
  }
  return min;
}

std::pair<Lane, Lane> GlobalInterface::doNothingLanes() {
  Lane current;
  current.push_back(ego_pose);
  return {current, current};
}

std::pair<Lane, Lane> GlobalInterface::getCurrentDesiredLane() {
  if (path.size() == 0){
    return doNothingLanes();
  } else if (path.size() == 1){
    return {path[0], path[0]};
  } else if (path[1].id == path[0].left_id || path[1].id == path[0].right_id) {
    // lane change
    return {path[0], path[1]};
  }
  auto current = path[0];

  // TEMPORARY:
  // Extend Current trajectory by 1 link
  current.insert(current.end(), path[1].begin(), path[1].end());

  // no lane change, follow the road
  return{current, current};
}

// find the length of `path`
double GlobalInterface::getPathLength(){
  double length = 0;
  if (path.size() == 0) return 0;

  for (int i = 0; i < path.size() - 1; i++){
    //continuation
    if (path[i].next.find(path[i+1].id) != path[i].next.end()){
      length += path[i+1].length();
    }
    // else Lane change, don't double count
  }
  length += path.back().length();
  return length;
}
bool GlobalInterface::planPath(){
  bool can_retry = false;
  auto cached_path = path;

  if (!path.empty()){
    can_retry = true;
  } else {
    Lane current = hdm_interface.getCurrentLane(ego_pose);
    path.push_back(current);
    can_retry = false;
  }
  try{
    // failure to find a path
    if (!breadthFirstSearch()){
      // retry if possible
      if (can_retry){
        path.clear();
        planPath();
      }
      // Failed, let upstream handle it with a reroute request
      return false;
    }
  } catch (const std::out_of_range& oor){
    // Did not get expected data from the HD map service. Handle and rethrow.
    path = cached_path;
    throw oor;
  }

  DEBUG_PRINT_PATH();

  return true;
}
Lane& GlobalInterface::queryLane(int lane_id) {
  if (topology.count(lane_id) == 0){
    topology = hdm_interface.updateLaneTopology(topology, lane_id, ego_pose);

#if debug
    std::cout << "Lane topology" << std::endl;
    for (auto& pair : topology){
      std::cout << "\t" << pair.first << ":" << pair.second << std::endl;
    }
#endif
  }
  return topology.at(lane_id);
}

// Traverse from path.back() to global_cmd_queue.top().link_id
bool GlobalInterface::breadthFirstSearch(){
  if (path.empty()){
    ROS_ERROR("BFS called with empty path");
    throw;
  }
  std::queue<int> q;
  std::unordered_map<int, int> pred;
  int destID = 0;
  auto& destinations = global_cmd_queue.front().link_id;

  // Standard BFS, searching for any link id
  int start_id = path.back().id;
  q.push(start_id);
  pred[start_id] = 0;

  // keep track of depth
  int currentDepth = 0, 
    elementsToDepthIncrease = 1, 
    nextElementsToDepthIncrease = 0;

  // edge case current link is the desired link
  if (destinations.find(path.back().start_link_id) != destinations.end()){
    // don't actually need ot do anything
    ROS_DEBUG_STREAM("Current link is a destination " << path.back());
    return true;
  }

  while (!q.empty()){
    int curID = q.front();
    Lane& curLane = queryLane(curID);
    q.pop();

    ROS_DEBUG_STREAM("Current " << curLane);

    // Always traverse "NEXT" before lane change
    std::vector<int> adj{curLane.next.begin(), curLane.next.end()};
    if (curLane.left_id != 0)
      adj.push_back(curLane.left_id);
    if (curLane.right_id != 0)
      adj.push_back(curLane.right_id);

    // Check depth
    nextElementsToDepthIncrease += adj.size();
    if (--elementsToDepthIncrease == 0) {
      if (Dynamic_Config.link_lookahead != -1 && ++currentDepth > Dynamic_Config.link_lookahead) return false;
      elementsToDepthIncrease = nextElementsToDepthIncrease;
      nextElementsToDepthIncrease = 0;
    }

    for (auto& nextID : adj){
    ROS_DEBUG_STREAM("\t Next: " << nextID);
      // not yet visited
      if (pred.count(nextID) == 0){

        ROS_DEBUG_STREAM("\t\t UNVISITED:" << nextID);

        pred[nextID] = curID;
        q.push(nextID);
        if (destinations.find(queryLane(nextID).start_link_id) != destinations.end()){

          ROS_DEBUG_STREAM("\t\t DESTINATION" << nextID);
          destID = nextID;
          break;
        }
      }
    }
    // Goal has been found
    if (destID != 0){
      break;
    }
  }

  // Goal has not been found
  if (destID == 0){
    return false;
  }
  // extract path from pred
  std::stack<int> crawl;
  while(pred.count(destID) != 0 && destID != start_id){
    crawl.push(destID);
    destID = pred[destID];
    // purposefully do not include the last predecessor
    // - it's the source ID and is already included in `path`
  }
  while (!crawl.empty()){
    int id = crawl.top();
    crawl.pop();
    path.push_back(queryLane(id));
  }

  if (global_cmd_queue.front().cmd == GlobalCommand::INSTRUCTION_TYPE::STOP){
    while (path.back().right_id != 0){
      path.push_back(queryLane(path.back().right_id));
    }
  }
  return true;
}
}
