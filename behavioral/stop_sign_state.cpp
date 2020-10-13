#include "stop_sign_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

StopSignState::StopSignState(StopSign sign, StopSignStateAction action):
    AbstractState(action), _sign{sign} {}

std::string StopSignState::serializeAction() const {
    switch(_action) {
        case StopSignStateAction::NON_BLOCKING:
            return "NON_BLOCKING";
        case StopSignStateAction::BLOCKING:
            return "BLOCKING";
        case StopSignStateAction::ACTIVE_BLOCKING:
            return "ACTIVE_BLOCKING";
        case StopSignStateAction::INACTIVE:
            return "INACTIVE";
        default:
            return "UNKNOWN, update StopSignState::serializeAction()";
    }
}

std::string StopSignState::serialize() const {
    return "";
}

StopSignState StopSignState::setSign(const StopSign& sign) {
    _sign = sign;
    return *this;
}

boost::optional<StopLine> StopSignState::getStopLine() const {
    boost::optional<path_planning::StopLine> stopLine = _sign.getStopLine();
    if (!stopLine) {
        return boost::none;
    }
    stopLine->obj = _sign;
    return stopLine;
}

const StopSign& StopSignState::getSign() const {
    return _sign;
}

StopSignState StopSignState::setTrajectoryReference(const geom::Polyline& line) {
    _trajectory_reference = line;
    return *this;
}

const geom::Polyline& StopSignState::getTrajectoryReference() const {
    return _trajectory_reference;
}

StopSignState StopSignState::setVehiclePose(const VehicleState &pose) {
    _vehicle_pose = pose;
    return *this;
}

const VehicleState& StopSignState::getVehiclePose() const {
    return _vehicle_pose;
}

void StopSignState::updateStopResult() {
    if (_action == StopSignStateAction::NON_BLOCKING) {
        return;
    }
    _stop_result.updateDistance(_vehicle_pose);
}

StopSignState StopSignState::setStopResult(const StopResult &result) {
    _stop_result = result;
    return *this;
}

const StopResult& StopSignState::getStopResult() const {
    return _stop_result;
}

StopSignState StopSignState::setActivatedTime(ros::Time time) {
    _activated_time = time;
    return *this;
}

ros::Time StopSignState::getActivatedTime() const {
    return _activated_time;
}