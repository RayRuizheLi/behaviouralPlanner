#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <exception>

#include <angles/angles.h>
#include "behavioral_planner.hpp"
#include "../DynamicNode/parameter_config.h"

namespace path_planning {

using local_planner::constants::stop_end_map_lines;

namespace behavioral_planner {

double current_speed_settings =
    Dynamic_Config.MAX_VELOCITY;  // Dynamic_Config.MAX_VELOCITY;

// How finely to partition the road for processing
const double BORDER_RESOLUTION = 0.5;
const double ROAD_PARTITION_DENSITY = 2;

const double VEHICLE_STOPPED_EPSILON = 1.0e-04;

// closest polyline to state and p (if p exists, otherwise closest polyline to state)
// returns -1 if no line exists
template <typename T>
int findClosestPolylineIndex(const std::vector<T>& lines,
                             const VehicleState& state,
                             const Point2d* p) {

  double closest_dist = std::numeric_limits<double>::max();
  int closest_line = -1;
  for (int i = 0; i < lines.size(); i++) {
    // ignore empty lines
    if (lines[i].empty()){
      continue;
    }

    auto dist = lines[i].distanceTo(state);

    if (p) {
      double pDist = lines[i].distanceTo(*p);
      if (pDist < 0.1) {
        // if it's the same line, ignore it.
        continue;
      }
      dist += pDist;
    }

    if (dist < closest_dist) {
      closest_dist = dist;
      closest_line = i;
    }
  }

  // std::cout << "RESULT: " << (*closest_line)[0] << " " <<
  // closest_line->back() << std::endl;
  return closest_line;
}

// closest polyline to state and p
//  T must be a default-constructable and movable subclass of Polyline
//  if lines is empty, returns an empty polyline
template <typename T>
T findClosestPolyline(const std::vector<T>& lines, const VehicleState& state,
                      const Point2d* p) {
  int idx = findClosestPolylineIndex(lines, state, p);
  if (idx == -1){
    T t;
    t.push_back(state);
    return t;
  }

  return lines[idx];
}


BlinkerState BehavioralPlanner::getBlinkerState(
  const VehicleState &ego_pose, const RoadLine& current, const RoadLine& target) {
  BlinkerState blinker_state = NO_BLINKER;
  auto global_cmd_queue = global_interface.getGlobalCommandQueue();

  // Turn on blinker if within 10m of intersection and turning
  if (!global_cmd_queue.empty() && global_cmd_queue.front().point.distanceTo(ego_pose) < 10){
    if (global_cmd_queue.front().cmd == GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN)
      blinker_state = RIGHT_BLINKER;
    if (global_cmd_queue.front().cmd == GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN)
      blinker_state = LEFT_BLINKER;
  }

  // Check Lane change
  if (abs(current.id) > abs(target.id)) {
    blinker_state = LEFT_BLINKER;
  }
  else if (abs(current.id) < abs(target.id)) {
    blinker_state = RIGHT_BLINKER;
  }

  return blinker_state;
}
#if 0
/*
 * ComputeTurnType: compute the desired lane change based on the desired lane IDs and the current lane
 */
const Lane* BehavioralPlanner::computeLaneChange(const Lane& currLane, const GlobalCommand::INSTRUCTION_TYPE cmd){

  const Lane* desiredLane = nullptr;
  int dir;
  int desiredLaneId = currLane.id;

  if (cmd != GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN &&
      cmd != GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN){
    return &currLane;
  }
  dir = cmd == GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN ? 1 : -1;
  desiredLaneId = currLane.id > 0 ? currLane.id + dir : currLane.id - dir;

  // search for desired lane
  for(auto& lane : topology){
    if (lane.id == desiredLaneId){
      desiredLane = &lane;
      break;
    }
  }

  if (!desiredLane){
    return &currLane;
  }
  return desiredLane;
}
#endif
/*
  ------------------------ DEPRECATED -----------------------------
 */
#if 0

std::vector<LineSegment> partition_road(const RoadLine& left,
                                        const RoadLine& right,
                                        double segment_separation) {
  std::vector<LineSegment> segments;
  segments.push_back(geom::LineSegment(left[0], right[0]));

  unsigned int left_index = 0;
  unsigned int right_index = 0;
  while (true) {
    if (left_index == left.size() - 1) {
      right_index++;
    } else if (right_index == right.size() - 1) {
      left_index++;
    } else {
      auto left_advance_length =
          (left[left_index + 1] - right[right_index]).length_squared();
      auto right_advance_length =
          (left[left_index] - right[right_index + 1]).length_squared();

      if (left_advance_length < right_advance_length) {
        left_index++;
      } else {
        right_index++;
      }
    }

    LineSegment segment(left[left_index], right[right_index]);
    if (left_index == left.size() - 1 && right_index == right.size() - 1) {
      segments.push_back(segment);
      break;
    } else if ((segment.center() - segments.back().center()).length() >=
               segment_separation) {
      segments.push_back(segment);
    }
  }

  return segments;
}

RoadLine findFurthestPolyline(const Point2d& p, const RoadLines& lines) {
  assert(!lines.empty());
  double furthest_dist = -1;
  const RoadLine* furthest_line = nullptr;
  for (auto& line : lines) {
    auto dist = line.distanceTo(p);
    if (dist > furthest_dist) {
      furthest_dist = dist;
      furthest_line = &line;
    }
  }
  return *furthest_line;
}

std::array<RoadLine, 2> findBorderingLines(const Point2d& p,
                                           const RoadLines& lines) {
  assert(lines.size() > 1);
  auto furthestBorder = findFurthestPolyline(p, lines);
  auto pointOnFurthestBorder = furthestBorder.closestElement(p);
  auto closestBorder = findFurthestPolyline(pointOnFurthestBorder, lines);
  return {{furthestBorder, closestBorder}};
}

std::array<RoadLine, 2> getBorders(const Environment& env) {
  auto roadLines = env.lines;
  auto rawBorders = findBorderingLines(env.start, roadLines);
  return {{rawBorders[0].subdivide(BORDER_RESOLUTION),
           rawBorders[1].subdivide(BORDER_RESOLUTION)}};
}

unsigned int closestRoadSegmentIndex(const Point2d& p,
                                     const std::vector<LineSegment>& segments) {
  double min_dist_squared = std::numeric_limits<double>::max();
  unsigned int closest_index = 0;
  for (unsigned int i = 0; i < segments.size(); i++) {
    auto dist_squared = (segments[i].center() - p).length_squared();
    if (dist_squared < min_dist_squared) {
      closest_index = i;
      min_dist_squared = dist_squared;
    }
  }

  return closest_index;
}

double distanceAlongRoadSegments(const Point2d& p1, const Point2d& p2,
                                 const std::vector<LineSegment>& segments) {
  unsigned int closest_index_to_p1 = closestRoadSegmentIndex(p1, segments);
  unsigned int closest_index_to_p2 = closestRoadSegmentIndex(p2, segments);

  if (closest_index_to_p1 == closest_index_to_p2) {
    return 0;
  }

  auto dir = closest_index_to_p1 < closest_index_to_p2 ? 1 : -1;
  double dist_travelled = 0;
  for (int i = closest_index_to_p1 + dir;
       dir * i <= dir * (int)closest_index_to_p2; i += dir) {
    dist_travelled +=
        dir * (segments[i].center() - segments[i - dir].center()).length();
  }

  return dist_travelled;
}

geom::LineSegment travelAlongRoad(const Point2d& p, RoadLine line, double dist,
                                  const Environment& env, bool* end_of_road) {
  static geom::LineSegment lastGoal;
  *end_of_road = true;
  long int closest_index = line.closestElementIndex(env.start);
  int direction = 1;
  long int end = line.size();
  double relative = 0;

  assert(line.size() > 1);
  if (closest_index < line.size() - 1) {
    relative = angles::normalize_angle_positive(
        (line[closest_index + 1] - line[closest_index]).angle() -
        env.start.heading);
  } else {
    relative = angles::normalize_angle_positive(
        (line[closest_index] - line[closest_index - 1]).angle() -
        env.start.heading);
  }
  if (relative > M_PI / 2 && relative < 3 * M_PI / 2) {
    direction = -1;
    end = -1;
  }

  double dist_travelled = 0;
  auto lookahead_point = line.back();
  closest_index += direction;
  while (closest_index != end) {
    // special case when it's the first loop, if the points in the polyline are
    // really far apart, then the dist travelled won't be accurate
    // we could also solve this by partitioning the polyline
    if (closest_index - direction == line.closestElementIndex(env.start)) {
      dist_travelled += (line[closest_index] - env.start).length();
    } else {
      dist_travelled +=
          (line[closest_index] - line[closest_index - direction]).length();
    }
    if (dist_travelled >= dist) {
      lookahead_point = line[closest_index];
      *end_of_road = false;
      break;
    }
    closest_index += direction;
  }

  auto l2 = findClosestPolyline(env.lines, env.start, &lookahead_point);
  auto p2 = l2.closestPoint(lookahead_point);

  StopLine tmpLine{lookahead_point, p2};

  // if the car is past the goal line, use the cache
  if (!tmpLine.queryRegion(env.start)) {
    return lastGoal;
  }

  lastGoal = {lookahead_point, p2};
  return {lookahead_point, p2};
}

GoalLine BehavioralPlanner::computeGoal(const Environment& env) {
  switch (state.get()) {
    case WAITING:
      return waiting(env);
    case DRIVING_GOAL_STOPPED:
      return driving_goal_stopped(env);
    case STOPPED_AND_READY:
    case DRIVING_GOAL_MAX_SPEED:
    case DRIVING_GOAL_FINISHED:
      return find_goal(env);
  }
}

/* FSM states */
GoalLine BehavioralPlanner::waiting(const Environment& env) {
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff_in_seconds = end - time_stopped;

  if (diff_in_seconds > MIN_STOP_WAIT_SECONDS) {
    state.set(STOPPED_AND_READY);
    return computeGoal(env);
  } else {
    return {env.start, env.start, 0.0, 0.0};
  }
}

GoalLine BehavioralPlanner::driving_goal_stopped(const Environment& env) {
  if (env.start.vel < VEHICLE_STOPPED_EPSILON &&
      stopline_goal.distanceTo(env.start) < 1.5) {
    last_goal = stopline_goal;
    has_last_goal = true;
    time_stopped = std::chrono::steady_clock::now();
    state.set(WAITING);
    return {env.start, env.start, 0.0, 0.0};
  }

  return find_goal(env);
}

/* Helper Functions */
GoalLine BehavioralPlanner::find_goal(const Environment& env) {
  double targetSpeed = Dynamic_Config.MAX_VELOCITY;

  /*
  bool shouldPark = false;
  for (auto trafficSign : env_msg.traffic_signs) {
    if trafficSign.
  }
  */

  if (env.parking_spots.size() > 0 && env.containsParkingSign()) {
    ParkingSpot parkingSpot = env.parking_spots[0];
    int endParkDistance = parkingSpot.length() * 2;
    if (Dynamic_Config.FORWARD_MOVEMENT_LOOKAHEAD / 2 > endParkDistance)
      endParkDistance = Dynamic_Config.FORWARD_MOVEMENT_LOOKAHEAD / 2;
    if (parkingSpot.p2.distanceTo(env.start) < endParkDistance) {
      return GoalLine(parkingSpot.getEndSegment(), 0, 0);
    } else if (parkingSpot.p1.distanceTo(env.start) <
               Dynamic_Config.FORWARD_MOVEMENT_LOOKAHEAD) {
      return GoalLine(parkingSpot.getEntrySegment(), 0, 0);
    }
  }

  if (env.lines.size() < 2) {
    if (!env.stop_lines.empty()) {
      return {env.stop_lines[0], 0.0, 0.0};
    }
    return {env.start, env.start, 0.0, 0.0};
  }
#if 0
  auto borders = getBorders(env);
  auto road_sections =
      partition_road(borders[0], borders[1], ROAD_PARTITION_DENSITY);

  auto stop_lines = env.stop_lines;
  if (!stop_lines.empty()) {
    double closest_distance = std::numeric_limits<double>::max();
    StopLine* closest_stopline_ahead = nullptr;

    for (auto& stop_line : stop_lines) {
      auto dist = distanceAlongRoadSegments(env.start, stop_line.center(),
                                            road_sections);
      if (has_last_goal && (stop_line.center() - last_goal.center()).length() < 1) {
        continue;
      }
      if (dist >= 0 && dist < closest_distance &&
          dist < Dynamic_Config.FORWARD_MOVEMENT_LOOKAHEAD) {
        closest_distance = dist;
        closest_stopline_ahead = &stop_line;
      }
    }

    if (closest_stopline_ahead != nullptr) {
      state.set(DRIVING_GOAL_STOPPED);
      stopline_goal = *closest_stopline_ahead;
      return GoalLine(*closest_stopline_ahead, 0, 0);
    }
  }
#endif

  if (env.traffic_signs.size() == 0)
    current_speed_settings = Dynamic_Config.MAX_VELOCITY;
  for (auto& traffic_sign : env.traffic_signs) {
    auto speed_limit_sign = dynamic_cast<SpeedLimitSign*>(traffic_sign.get());
    if (speed_limit_sign != nullptr) {
      current_speed_settings = speed_limit_sign->getSpeed();
    }
  }

  bool end_of_road = true;
  // std::cout << "Planning" << std::endl;
  auto target = travelAlongRoad(
      env.start, findClosestPolyline(env.lines, env.start),
      Dynamic_Config.FORWARD_MOVEMENT_LOOKAHEAD, env, &end_of_road);
  // std::cout << "Done Planning" << std::endl;
  state.set(end_of_road && stop_end_map_lines ? DRIVING_GOAL_FINISHED
                                              : DRIVING_GOAL_MAX_SPEED);
  return GoalLine(
      target, end_of_road && stop_end_map_lines ? 0 : current_speed_settings,
      0);
}
#endif

}  // namespace behavioral_planner
}  // namespace path_planning
