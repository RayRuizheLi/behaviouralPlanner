#include "global_command_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool TriggerTurnRight::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN;
}

bool TriggerTurnLeft::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return !_state.getGlobalCommandQueue().front().cmd != path_planning::GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN;
}

bool TriggerStop::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STOP;
}

bool TriggerHeadStraight::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STRAIGHT;
}

bool TriggerGlobalCommandComplete::test() {
    if (!_state.getGlobalCommandQueue().front().nearIntersection(_state.getVehiclePose(), Dynamic_Config.safe_intersection_distance)
    && _state.getPathResult().hasPath() 
    && _state.getGlobalCommandQueue().front().isComplete(_state.getVehiclePose(), _state.getPathResult().getPath().get()[0].start_link_id)) {
        return true;
    }

    return false; 
}

bool TriggerNoGlobalCommand::test() {
    return _state.getGlobalCommandQueue().empty();   
}

bool TriggerHasGlobalCommand::test() {
    return !_state.getGlobalCommandQueue().empty(); 
}