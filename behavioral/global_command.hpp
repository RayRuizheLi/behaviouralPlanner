#pragma once

#include "../common/point2d.hpp"
#include "../common/environment.hpp"
#include "../common/vehicle_state.hpp"
#include <queue>
#include <set>

namespace path_planning{

  class invalid_instruction{};

  struct GlobalCommand {
    enum INSTRUCTION_TYPE { NONE, LEFT_TURN, RIGHT_TURN, STRAIGHT, STOP };

    GlobalCommand() : cmd(NONE) {}
    GlobalCommand(geom::Point2d p, INSTRUCTION_TYPE cmd, std::set<int64_t> link_id) :
        point{p}, cmd{cmd}, link_id{link_id} {}
    
    /* x,y coordinate where the command should be excuted: 
     *   -intersection position for TURN commands
     *   -destination position for STOP 
     */
    geom::Point2d point;
    
    /* Instruction to be executed at the given point */
    GlobalCommand::INSTRUCTION_TYPE cmd;
    
    /* List of HD map Link IDs.
     * GlobalInterface will plan a lanenet path to any lane with a link ID in this list
     * We use a list of link ids since correspondence between an SD map road ID 
     * to HD map link ID is not unique. Global Planner finds the SD map ID, 
     * Then uses a mapping to determine the corresponding list of link IDs.
     * */
     
    std::set<int64_t> link_id;

    bool isComplete(path_planning::VehicleState ego_pose, int cur_link_id);
    bool nearIntersection(VehicleState& _vehicle_pos, double safe_intersection_distance) const;

    bool seen = false;
  };


inline std::ostream& operator<<(std::ostream& os, const GlobalCommand::INSTRUCTION_TYPE& inst){
  switch (inst){
    case GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN:
      os << "LEFT_TURN";
      break;
    case GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN:
      os << "RIGHT_TURN";
      break;
    case GlobalCommand::INSTRUCTION_TYPE::STRAIGHT:
      os << "STRAIGHT";
      break;
    case GlobalCommand::INSTRUCTION_TYPE::STOP:
      os << "STOP";
      break;
    default:
      os << "NONE";
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const GlobalCommand& cmd){
  os << "{ Global Command: " << cmd.cmd << "; pos " << cmd.point << " " << "IDs: ";
  for (auto& id : cmd.link_id){
    os << id << " ";
  }
  os << "}";
  return os;
}


  using GlobalCommandQueue = std::queue<path_planning::GlobalCommand>;
}
