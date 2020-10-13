#include "stop_sign_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool StopSignPresentOnRouteTrigger::test() {

    if (_state.getSign().getStopLine() == boost::none) {
        return false;
    }

    StopResult tryStop = StopResult::tryCreateWithStopLine(
        _state.getTrajectoryReference(), _state.getVehiclePose(), _state.getSign().getStopLine().get());

    if (!tryStop.hasStop()) {
        return false;
    }

    stop_result = tryStop;

    return true;
}

bool StopSignActivatedTrigger::test() {
    if (_state.getVehiclePose().distanceTo(_state.getStopResult().getStopPoint()) > Dynamic_Config.groups.behavioral_planning.STOP_SIGN_DISTANCE_EPSILON
        || _state.getVehiclePose().vel > 0.2) {
        return false;
    }

    return true;
}

bool StopSignDeactivatedTrigger::test() {
    if (_state.getVehiclePose().distanceTo(_state.getStopResult().getStopPoint()) >= Dynamic_Config.groups.behavioral_planning.STOP_SIGN_DISTANCE_EPSILON
        || _state.getVehiclePose().vel > 0.2) {
        return true;
    }

    return false;
}

bool StopSignExpiredTrigger::test() {
    ros::Duration activeDuration = ros::Time::now() - _state.getActivatedTime();
    ros::Duration stopTime(Dynamic_Config.groups.behavioral_planning.STOP_SIGN_TIME);

    return activeDuration >= stopTime;
}