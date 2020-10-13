#include "stop_sign_state_machine.hpp"

using namespace path_planning::behavioral;

StopSignStateMachine::StopSignStateMachine(
    StopSign initial_sign): 
        AbstractStateMachine(createTransitionGraph(), 
        std::move(std::unique_ptr<StopSignTriggerFactory>(new StopSignTriggerFactory())), 
        {initial_sign, StopSignStateAction::NON_BLOCKING}) {
            ROS_INFO_STREAM("[" << getName() << "]\n" << "Created with initial state action: " << _state.serializeAction());
        }

AbstractTransitionGraph<StopSignStateAction, StopSignTriggerID> StopSignStateMachine::createTransitionGraph() {
    AbstractTransitionGraph<StopSignStateAction, StopSignTriggerID> g;

    g.addState(StopSignStateAction::NON_BLOCKING, 
        {std::make_pair(StopSignTriggerID::PRESENT_ON_ROUTE, StopSignStateAction::BLOCKING)});

    g.addState(StopSignStateAction::BLOCKING, 
        {std::make_pair(StopSignTriggerID::ACTIVATED, StopSignStateAction::ACTIVE_BLOCKING)});

    g.addState(StopSignStateAction::ACTIVE_BLOCKING, 
        {std::make_pair(StopSignTriggerID::DEACTIVATED, StopSignStateAction::BLOCKING),
        std::make_pair(StopSignTriggerID::EXPIRED, StopSignStateAction::INACTIVE)});

    return g;
}

bool StopSignStateMachine::transition(const StopSignTrigger &trigger) {
    // Let base class function to graph trasition and set new ego action
    if (!AbstractStateMachine::transition(trigger)) {
        return false;
    }

    // Process trigger (e.g. store stopping distance in state)
    switch (trigger.getID()) {
        case StopSignTriggerID::PRESENT_ON_ROUTE:
        {
            const StopSignPresentOnRouteTrigger& onRouteTrigger = dynamic_cast<const StopSignPresentOnRouteTrigger&>(trigger);
            _state.setStopResult(onRouteTrigger.stop_result);
            break;
        }
        case StopSignTriggerID::ACTIVATED:
        {
            _state.setActivatedTime(ros::Time::now());
            break;
        }
        default:
            return true;
    }

    return true;

}

void StopSignStateMachine::cycle() {
    _state.updateStopResult();

    // Let base class handle doing graph cycle and calling transition
    AbstractStateMachine::cycle();
}

std::string StopSignStateMachine::getName() const {
    return "StopSignStateMachine" + std::to_string(_state.getSign().id);
}