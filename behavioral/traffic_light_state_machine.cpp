#include "traffic_light_state_machine.hpp"

using namespace path_planning::behavioral;

TrafficLightStateMachine::TrafficLightStateMachine(
    path_planning::TrafficLight initial_light): 
        AbstractStateMachine(createTransitionGraph(), 
        std::move(std::unique_ptr<TrafficLightTriggerFactory>(new TrafficLightTriggerFactory())), 
        {initial_light, TrafficLightStateAction::NON_BLOCKING}) {
            ROS_INFO_STREAM("[" << getName() << "]\n" << "Created with initial state action: " << _state.serializeAction());
        }

AbstractTransitionGraph<TrafficLightStateAction, TrafficLightTriggerID> TrafficLightStateMachine::createTransitionGraph() {
    AbstractTransitionGraph<TrafficLightStateAction, TrafficLightTriggerID> g;

    g.addState(TrafficLightStateAction::NON_BLOCKING, 
        {std::make_pair(TrafficLightTriggerID::PRESENT_ON_ROUTE, TrafficLightStateAction::BLOCKING)});

    g.addState(TrafficLightStateAction::BLOCKING, 
        {std::make_pair(TrafficLightTriggerID::REMOVED, TrafficLightStateAction::NON_BLOCKING)});

    return g;
}

std::string TrafficLightStateMachine::getName() const {
    return "TrafficLightStateMachine" + std::to_string(_state.getLightStatus().id);
}

bool TrafficLightStateMachine::transition(const TrafficLightTrigger &trigger) {
    // Let base class function to graph trasition and set new ego action
    if (!AbstractStateMachine::transition(trigger)) {
        return false;
    }

    // Process trigger (e.g. store stopping distance in state)
    switch (trigger.getID()) {
        case TrafficLightTriggerID::PRESENT_ON_ROUTE:
        {
            const TrafficLightPresentOnRouteTrigger& onRouteTrigger = dynamic_cast<const TrafficLightPresentOnRouteTrigger&>(trigger);
            _state.setStopResult(onRouteTrigger.stop_result);
        }
        default:
            return true;
    }

    return true;

}

void TrafficLightStateMachine::cycle() {
    _state.updateStopResult();

    // Let base class handle doing graph cycle and calling transition
    AbstractStateMachine::cycle();
}