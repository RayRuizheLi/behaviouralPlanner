#include "pedestrian_state_machine.hpp"

using namespace path_planning::behavioral;

PedestrianStateMachine::PedestrianStateMachine(
    Pedestrian initial_ped): 
        AbstractStateMachine(createTransitionGraph(), 
        std::move(std::unique_ptr<PedestrianTriggerFactory>(new PedestrianTriggerFactory())), 
        {initial_ped, PedestrianStateAction::IGNORED}) {
            ROS_INFO_STREAM("[" << getName() << "]\n" << "Created with initial state action: " << _state.serializeAction());
        }

AbstractTransitionGraph<PedestrianStateAction, PedestrianTriggerID> PedestrianStateMachine::createTransitionGraph() {
    AbstractTransitionGraph<PedestrianStateAction, PedestrianTriggerID> g;

    g.addState(PedestrianStateAction::IGNORED, 
        {std::make_pair(PedestrianTriggerID::IN_ROI, PedestrianStateAction::ACTIVE)});

    g.addState(PedestrianStateAction::NON_ACTIVE, 
        {std::make_pair(PedestrianTriggerID::MOVING, PedestrianStateAction::ACTIVE)});

    g.addState(PedestrianStateAction::ACTIVE, 
        {std::make_pair(PedestrianTriggerID::NOT_MOVING, PedestrianStateAction::PAUSED),
        std::make_pair(PedestrianTriggerID::NOT_IN_ROI, PedestrianStateAction::IGNORED)});

    g.addState(PedestrianStateAction::PAUSED, 
        {std::make_pair(PedestrianTriggerID::STOPPED, PedestrianStateAction::NON_ACTIVE),
        std::make_pair(PedestrianTriggerID::MOVING, PedestrianStateAction::ACTIVE)});

    return g;
}

bool PedestrianStateMachine::transition(const PedestrianTrigger &trigger) {
    // Let base class function to graph trasition and set new ego action
    if (!AbstractStateMachine::transition(trigger)) {
        return false;
    }

    // Process trigger (e.g. store stopping distance in state)
    switch (trigger.getID()) {
        case PedestrianTriggerID::IN_ROI:
        {
            const PedestrianInRoITrigger& inRoITrigger = dynamic_cast<const PedestrianInRoITrigger&>(trigger);
            _state.setStopResult(inRoITrigger.stop_result);
            break;
        }
        case PedestrianTriggerID::NOT_MOVING:
        {
            _state.setPausedTime(ros::Time::now());
            break;
        }
        default:
            return true;
    }

    return true;

}

void PedestrianStateMachine::cycle() {
    _state.updateStopResult();

    // Let base class handle doing graph cycle and calling transition
    AbstractStateMachine::cycle();
}

std::string PedestrianStateMachine::getName() const {
    return "PedestrianStateMachine" + std::to_string(_state.getPedestrianObj().id);
}