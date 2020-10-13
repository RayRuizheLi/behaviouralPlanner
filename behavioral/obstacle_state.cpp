#include "obstacle_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

ObstacleState::ObstacleState(Obstacle obj, ObstacleStateAction action):
    AbstractState(action), _obj{obj} {}

std::string ObstacleState::serializeAction() const {
    switch(_action) {
        case ObstacleStateAction::IGNORED:
            return "IGNORED";
        case ObstacleStateAction::MOVING:
            return "MOVING";
        case ObstacleStateAction::STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN, update ObstacleState::serializeAction()";
    }
}

std::string ObstacleState::serialize() const {
    return "";
}

ObstacleState ObstacleState::setObstacle(const Obstacle& obj) {
    _obj = obj;
    return *this;
}

const Obstacle& ObstacleState::getObstacle() const {
    return _obj;
}

ObstacleState ObstacleState::setTrajectoryReference(const geom::Polyline& line) {
    _trajectory_reference = line;
    return *this;
}

const geom::Polyline& ObstacleState::getTrajectoryReference() const {
    return _trajectory_reference;
}

ObstacleState ObstacleState::setVehiclePose(const VehicleState &pose) {
    _vehicle_pose = pose;
    return *this;
}

const VehicleState& ObstacleState::getVehiclePose() const {
    return _vehicle_pose;
}

void ObstacleState::updateStopResult() {
    if (_action != ObstacleStateAction::MOVING) {
        return;
    }
    _stop_result.updateDistance(_vehicle_pose);
}

ObstacleState ObstacleState::setStopResult(const StopResult &result) {
    _stop_result = result;
    return *this;
}

const StopResult& ObstacleState::getStopResult() const {
    return _stop_result;
}