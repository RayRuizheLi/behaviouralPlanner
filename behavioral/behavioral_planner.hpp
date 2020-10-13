#pragma once

#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <deque>

#include "DynamicNode/parameter_config.h"
#include "common/environment.hpp"
#include "common/goal_line.hpp"
#include "common/lane.hpp"
#include "common/line2d.hpp"
#include "common/line_segment.hpp"
#include "common/object.hpp"
#include "common/point2d.hpp"
#include "common/polyline.hpp"
#include "common/road_data.hpp"
#include "common/speed_limit_sign.hpp"
#include "common/stopline.hpp"
#include "common/ros_msgs.hpp"
#include "local_planning/includes/constants.hpp"
#include "global_command.hpp"
#include "blinker_state.hpp"

#include "global_interface.hpp"

namespace path_planning {
namespace behavioral_planner {

using geom::Line2d;
using geom::LineSegment;
using geom::Point2d;
using geom::Polyline;

struct global_reroute {};
struct invalid_turn {};
struct no_lane_available {
  no_lane_available(Lane l) : currLane(l) {}
  Lane currLane;
};

template <typename T>
int findClosestPolylineIndex(const std::vector<T>& lines,
                             const VehicleState& state,
                             const Point2d* p = nullptr);

template <typename T>
T findClosestPolyline(const std::vector<T>& lines,
                      const VehicleState& state,
                      const Point2d* p = nullptr);


static const std::chrono::duration<double> MIN_STOP_WAIT_SECONDS(2);
extern double
    current_speed_settings;  // local_planner::constants::MAX_VELOCITY;

// FSM States
enum BehaviouralState {
  WAITING = 0,
  STOPPED_AND_READY = 1,
  DRIVING_GOAL_MAX_SPEED = 2,
  DRIVING_GOAL_STOPPED = 3,
  DRIVING_GOAL_FINISHED = 4,
  MAX_SPEED_5MPH = 5,
  MAX_SPEED_10MPH = 6,
  MAX_SPEED_15MPH = 7,
  MAX_SPEED_20MPH = 8,
  MAX_SPEED_25MPH = 9,
  PARKING = 10,
};

class BehavioralPlannerStateManager {
 private:
  BehaviouralState state{STOPPED_AND_READY};

 public:
  static std::string stringForState(BehaviouralState state) {
    switch (state) {
      case WAITING:
        return "WAITING";
      case STOPPED_AND_READY:
        return "STOPPED_AND_READY";
      case DRIVING_GOAL_MAX_SPEED:
        return "DRIVING_GOAL_MAX_SPEED";
      case DRIVING_GOAL_STOPPED:
        return "DRIVING_GOAL_STOPPED";
      case DRIVING_GOAL_FINISHED:
        return "DRIVING_GOAL_FINISHED";
      case PARKING:
        return "PARKING";
      default:
        return "UNKNOWN";
    }
  }

  std::string str() const { return stringForState(state); }

  BehaviouralState get() const { return state; }

  void set(BehaviouralState newState) {
    if (newState != state) {
      std::cerr << "State change: " << stringForState(state) << " → "
                << stringForState(newState) << std::endl;
    }
    state = newState;
  }
};

class BehavioralPlanner {
  GlobalInterface& global_interface;

 public:
  BehavioralPlanner(GlobalInterface& global_interface) : global_interface(global_interface) {}

  BlinkerState getBlinkerState(
    const VehicleState &ego_pose, const RoadLine &current, const RoadLine &target);

/* --- Deprecated --- */
  /**
   Computes and assigns goal based on current car state (internal state machine)
   and envioronment data
   */
  std::pair<OccupiableObject, bool> getStopState();
  GoalLine computeGoal(const Environment &env);

  BehaviouralState currentState() { return state.get(); }

 private:
  /* Deprecated */

  BehavioralPlannerStateManager state;

  StopLine stopline_goal;
  bool has_last_goal = false;
  StopLine last_goal;
  std::chrono::steady_clock::time_point time_stopped;

  // States in the FSM
  GoalLine waiting(const Environment &env);
  GoalLine driving_goal_stopped(const Environment &env);

  // Helpers functions
  GoalLine find_goal(const Environment &env);
};

}  // namespace behavioral_planner
}  // namespace path_planning
