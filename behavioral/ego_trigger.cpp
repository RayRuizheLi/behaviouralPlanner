#include "ego_trigger.hpp"

using namespace path_planning::behavioral;

bool TriggerLeftLaneChange::test() {
    return _state.getLatestDesiredLatDev() - _state.getLatestLatDev() >= Dynamic_Config.groups.behavioral_planning.LANE_CHANGE_LAT_DEV_CONDITION;
}

bool TriggerRightLaneChange::test() {
    return _state.getLatestDesiredLatDev() - _state.getLatestLatDev() <= -1*Dynamic_Config.groups.behavioral_planning.LANE_CHANGE_LAT_DEV_CONDITION;
}

bool TriggerLaneStabilized::test() {
    std::deque<double> curr = _state.getLatDevHist();
    std::deque<double> desired = _state.getDesiredLatDevHist();
    auto curr_it = curr.begin();
    auto desired_it = desired.begin();
    for (unsigned int i = 0; i < Dynamic_Config.groups.behavioral_planning.LANE_STABILIZED_HISTORY && curr_it != curr.end() && desired_it != desired.end(); i++, curr_it++, desired_it++) {
        if (abs(*curr_it - *desired_it) > Dynamic_Config.LANE_STABILIZED_EPSILON) {
            return false;
        }
    }
    return true;
}

bool TriggerStopLineOnRoute::test() {
    return _state.getWorldStopResult().hasStop();
}

bool TriggerNoStopLineOnRoute::test() {
    return !_state.getWorldStopResult().hasStop();
}

bool TriggerNoGlobalCommands::test() {
    return _state.getGlobalCommand().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::NONE;
}

bool TriggerHasGlobalCommands::test() {
    return _state.getGlobalCommand().cmd != path_planning::GlobalCommand::INSTRUCTION_TYPE::NONE;
}