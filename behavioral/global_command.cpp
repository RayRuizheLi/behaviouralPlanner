#include "global_command.hpp"

namespace path_planning {

    bool GlobalCommand::isComplete(path_planning::VehicleState ego_pose, int cur_link_id){
        if (cmd == INSTRUCTION_TYPE::STOP){
          return ego_pose.distanceTo(point) < 5;
        }
        return link_id.count(cur_link_id) != 0;
    }

    bool GlobalCommand::nearIntersection(VehicleState& _vehicle_pose, double safe_intersection_distance) const {
        if (cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN ||
            cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN ||
            cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STRAIGHT){
            return (_vehicle_pose.distanceTo(point) < safe_intersection_distance);
        } else {
            return false;
        }
    }
  
}
