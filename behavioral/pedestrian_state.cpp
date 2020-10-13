#include "pedestrian_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

PedestrianState::PedestrianState(Pedestrian ped_obj, PedestrianStateAction action):
    AbstractState(action), _ped_obj{ped_obj} {}

std::string PedestrianState::serializeAction() const {
    switch(_action) {
        case PedestrianStateAction::IGNORED:
            return "IGNORED";
        case PedestrianStateAction::NON_ACTIVE:
            return "NON_ACTIVE";
        case PedestrianStateAction::ACTIVE:
            return "ACTIVE";
        case PedestrianStateAction::PAUSED:
            return "PAUSED";
        default:
            return "UNKNOWN, update PedestrianState::serializeAction()";
    }
}

std::string PedestrianState::serialize() const {
    return "";
}

PedestrianState PedestrianState::setPedestrianObj(const Pedestrian& ped_obj) {
    _ped_obj = ped_obj;
    return *this;
}

const Pedestrian& PedestrianState::getPedestrianObj() const {
    return _ped_obj;
}

PedestrianState PedestrianState::setTrajectoryReference(const geom::Polyline& line) {
    _trajectory_reference = line;
    return *this;
}

const geom::Polyline& PedestrianState::getTrajectoryReference() const {
    return _trajectory_reference;
}

PedestrianState PedestrianState::setVehiclePose(const VehicleState &pose) {
    _vehicle_pose = pose;
    return *this;
}

const VehicleState& PedestrianState::getVehiclePose() const {
    return _vehicle_pose;
}

void PedestrianState::updateStopResult() {
    if (!(_action == PedestrianStateAction::ACTIVE || _action == PedestrianStateAction::PAUSED)) {
        return;
    }
    _stop_result.updateDistance(_vehicle_pose);
}

PedestrianState PedestrianState::setStopResult(const StopResult &result) {
    _stop_result = result;
    return *this;
}

const StopResult& PedestrianState::getStopResult() const {
    return _stop_result;
}

PedestrianState PedestrianState::setPausedTime(ros::Time time) {
    _paused_time = time;
    return *this;
}

ros::Time PedestrianState::getPausedTime() const {
    return _paused_time;
}