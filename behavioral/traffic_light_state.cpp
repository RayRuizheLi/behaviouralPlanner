#include "traffic_light_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

TrafficLightState::TrafficLightState(path_planning::TrafficLight light, TrafficLightStateAction action):
    AbstractState(action), _light{light} {}

std::string TrafficLightState::serializeAction() const {
    switch(_action) {
        case TrafficLightStateAction::NON_BLOCKING:
            return "NON_BLOCKING";
        case TrafficLightStateAction::BLOCKING:
            return "BLOCKING";
        default:
            return "UNKNOWN, update TrafficLightState::serializeAction()";
    }
}

std::string TrafficLightState::serialize() const {
    return "";
}

TrafficLightState TrafficLightState::setLightStatus(path_planning::TrafficLight light) {
    _light = light;
    return *this;
}

path_planning::TrafficLight TrafficLightState::getLightStatus() const {
    return _light;
}

TrafficLightState TrafficLightState::setGlobalCommandQueue(path_planning::GlobalCommandQueue cmds) {
    _globalCommandQueue = cmds;
    return *this;
}

path_planning::GlobalCommandQueue TrafficLightState::getGlobalCommandQueue() const {
    return _globalCommandQueue;
}

boost::optional<path_planning::StopLine> TrafficLightState::getStoppingLine() const {
    boost::optional<path_planning::StopLine> stopLine = _light.tryGetStopLine();
    if (!stopLine) {
        return boost::none;
    }
    stopLine->obj = _light;
    return stopLine;
}

TrafficLightState TrafficLightState::setTrajectoryReference(const geom::Polyline& line) {
    _curr_trajectory_reference = line;
    return *this;
}


const geom::Polyline& TrafficLightState::getTrajectoryReference() const {
    return _curr_trajectory_reference;
}

TrafficLightState TrafficLightState::setVehiclePose(const VehicleState &pose) {
    _vehicle_pose = pose;
    return *this;
}

const VehicleState& TrafficLightState::getVehiclePose() const {
    return _vehicle_pose;
}

TrafficLightState TrafficLightState::setCurrentLane(const Lane &lane) {
    _current_lane = lane;
    return *this;
}

const Lane& TrafficLightState::getCurrentLane() const {
    return _current_lane;
}

TrafficLightState TrafficLightState::setStopResult(const StopResult &result) {
    _stop_result = result;
    return *this;
}

const StopResult& TrafficLightState::getStopResult() const {
    return _stop_result;
}

void TrafficLightState::updateStopResult() {
    if (_action != TrafficLightStateAction::BLOCKING) {
        return;
    }
    _stop_result.updateDistance(_vehicle_pose);
}

