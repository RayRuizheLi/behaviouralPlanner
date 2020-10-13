#include "pedestrian_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

/** 
 * Currently path planning is not given information about the crossing lanes in the world, 
 * so whenever a pedestrian comes close to our reference line, we stop for it, even if it is not 
 * near a crossing lane
*/
bool PedestrianInRoITrigger::test() {
    StopResult stopResult = StopResult::tryCreateWithDynamicObject(
        _state.getTrajectoryReference(), _state.getVehiclePose(), _state.getPedestrianObj());

    if (!stopResult.hasStop()) {
        return false;
    }

    stop_result = stopResult;

    return true;
}

bool PedestrianMovingTrigger::test() {
    return _state.getPedestrianObj().getVelocityMagnitude() >= Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool PedestrianNotMovingTrigger::test() {
    return _state.getPedestrianObj().getVelocityMagnitude() < Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

/** 
 * Currently path planning is not given information about the crossing lanes in the world, 
 * so we assume all pedestrians are near a crossing lane
*/
bool PedestrianNotInRoI::test() {
    return false;
}

bool PedestrianStoppedTrigger::test() {
    ros::Duration pausedDuration = ros::Time::now() - _state.getPausedTime();
    ros::Duration pausedTime(Dynamic_Config.groups.behavioral_planning.OBJECT_STOPPED_TIMER);

    return pausedDuration >= pausedTime;
}
