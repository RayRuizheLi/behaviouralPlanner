#include "obstacle_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool ObstacleInRoITrigger::test() {
    StopResult stopResult = StopResult::tryCreateWithDynamicObject(
        _state.getTrajectoryReference(), _state.getVehiclePose(), _state.getObstacle());

    if (!stopResult.hasStop()) {
        return false;
    }

    stop_result = stopResult;

    return true;
}

bool ObstacleMovingTrigger::test() {
    return _state.getObstacle().getVelocityMagnitude() >= Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool ObstacleStoppedTrigger::test() {
    return _state.getObstacle().getVelocityMagnitude() < Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool ObstacleNotInRoI::test() {
    return _state.getObstacle().distanceTo(_state.getTrajectoryReference().closestPoint(_state.getObstacle().center)) > Dynamic_Config.groups.behavioral_planning.OBJECT_ROI_RADIUS;
}