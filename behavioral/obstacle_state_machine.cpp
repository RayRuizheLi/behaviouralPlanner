#include "obstacle_state_machine.hpp"

using namespace path_planning::behavioral;

ObstacleStateMachine::ObstacleStateMachine(
    Obstacle initial_obj): 
        AbstractStateMachine(createTransitionGraph(), 
        std::move(std::unique_ptr<ObstacleTriggerFactory>(new ObstacleTriggerFactory())), 
        {initial_obj, ObstacleStateAction::IGNORED}) {
            ROS_INFO_STREAM("[" << getName() << "]\n" << "Created with initial state action: " << _state.serializeAction());
        }

AbstractTransitionGraph<ObstacleStateAction, ObstacleTriggerID> ObstacleStateMachine::createTransitionGraph() {
    AbstractTransitionGraph<ObstacleStateAction, ObstacleTriggerID> g;

    g.addState(ObstacleStateAction::IGNORED, 
        {std::make_pair(ObstacleTriggerID::IN_ROI, ObstacleStateAction::MOVING)});

    g.addState(ObstacleStateAction::MOVING, 
        {std::make_pair(ObstacleTriggerID::STOPPED, ObstacleStateAction::STOPPED),
        std::make_pair(ObstacleTriggerID::NOT_IN_ROI, ObstacleStateAction::IGNORED)});

    g.addState(ObstacleStateAction::STOPPED, 
        {std::make_pair(ObstacleTriggerID::MOVING, ObstacleStateAction::MOVING)});

    return g;
}

bool ObstacleStateMachine::transition(const ObstacleTrigger &trigger) {
    // Let base class function to graph trasition and set new ego action
    if (!AbstractStateMachine::transition(trigger)) {
        return false;
    }

    // Process trigger (e.g. store stopping distance in state)
    switch (trigger.getID()) {
        case ObstacleTriggerID::IN_ROI:
        {
            const ObstacleInRoITrigger& inRoITrigger = dynamic_cast<const ObstacleInRoITrigger&>(trigger);
            _state.setStopResult(inRoITrigger.stop_result);
            break;
        }
        default:
            return true;
    }

    return true;

}

void ObstacleStateMachine::cycle() {
    _state.updateStopResult();

    // Let base class handle doing graph cycle and calling transition
    AbstractStateMachine::cycle();
}

std::string ObstacleStateMachine::getName() const {
    return "ObstacleStateMachine" + std::to_string(_state.getObstacle().id);
}