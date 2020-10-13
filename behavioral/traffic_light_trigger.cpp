#include "traffic_light_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool TrafficLightPresentOnRouteTrigger::test() {
    if (_state.getGlobalCommandQueue().empty()) {
        return false;
    }
    path_planning::GlobalCommand nextCommand = _state.getGlobalCommandQueue().front();
    bool affectsCmd = 
        ((_state.getLightStatus().right_state == path_planning::TrafficLight::TrafficLightState::RED)
            && nextCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN)

        || ((_state.getLightStatus().forward_state == path_planning::TrafficLight::TrafficLightState::RED)
            && nextCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STRAIGHT)

        || ((_state.getLightStatus().left_state == path_planning::TrafficLight::TrafficLightState::RED)
            && nextCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN);

    if (!affectsCmd) {
        return false;
    }

    if (_state.getStoppingLine() == boost::none) {
        return false;
    }

    StopResult tryStop = StopResult::tryCreateWithStopLine(
        _state.getTrajectoryReference(), _state.getVehiclePose(), _state.getStoppingLine().get());

    if (!tryStop.hasStop()) {
        return false;
    }

    stop_result = tryStop;

    return true;

}

bool TrafficLightRemovedTrigger::test() {
    if (_state.getGlobalCommandQueue().empty()) {
        return true;
    }
    path_planning::GlobalCommand nextCommand = _state.getGlobalCommandQueue().front();
    return 
        ((_state.getLightStatus().right_state == path_planning::TrafficLight::TrafficLightState::GREEN)
            && nextCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN)

        || ((_state.getLightStatus().forward_state == path_planning::TrafficLight::TrafficLightState::GREEN)
            && nextCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STRAIGHT)

        || ((_state.getLightStatus().left_state == path_planning::TrafficLight::TrafficLightState::GREEN)
            && nextCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN);
}
