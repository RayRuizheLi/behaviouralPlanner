#include "global_command_state_machine.hpp"

using namespace path_planning::behavioral;

GlobalCommandStateMachine::GlobalCommandStateMachine(): 
        AbstractStateMachine(createTransitionGraph(), 
        std::move(std::unique_ptr<GlobalCommandTriggerFactory>(new GlobalCommandTriggerFactory())), 
        {GlobalCommandStateAction::WAITING_FOR_GLOBAL_COMMAND}) {
            ROS_INFO_STREAM("[" << getName() << "]\n" << "Created with initial state action: " << _state.serializeAction());
        }

AbstractTransitionGraph<GlobalCommandStateAction, GlobalCommandTriggerID> GlobalCommandStateMachine::createTransitionGraph() {
    AbstractTransitionGraph<GlobalCommandStateAction, GlobalCommandTriggerID> g;

    g.addState(GlobalCommandStateAction::DONE, {
        std::make_pair(GlobalCommandTriggerID::HEAD_STRAIGHT, GlobalCommandStateAction::HEADING_STRAIGHT),
        std::make_pair(GlobalCommandTriggerID::TURN_LEFT, GlobalCommandStateAction::TURNING_LEFT),
        std::make_pair(GlobalCommandTriggerID::TURN_RIGHT, GlobalCommandStateAction::TURNING_RIGHT),
        std::make_pair(GlobalCommandTriggerID::STOP, GlobalCommandStateAction::STOPPING),
        std::make_pair(GlobalCommandTriggerID::NO_GLOBAL_COMMAND, GlobalCommandStateAction::WAITING_FOR_GLOBAL_COMMAND)
    });

    g.addState(GlobalCommandStateAction::TURNING_LEFT, {
        std::make_pair(GlobalCommandTriggerID::COMMAND_COMPLETE, GlobalCommandStateAction::DONE)
    });

    g.addState(GlobalCommandStateAction::TURNING_RIGHT, {
        std::make_pair(GlobalCommandTriggerID::COMMAND_COMPLETE, GlobalCommandStateAction::DONE)
    });

    g.addState(GlobalCommandStateAction::HEADING_STRAIGHT, {
        std::make_pair(GlobalCommandTriggerID::COMMAND_COMPLETE, GlobalCommandStateAction::DONE)
    });

    g.addState(GlobalCommandStateAction::STOPPING, {
        std::make_pair(GlobalCommandTriggerID::COMMAND_COMPLETE, GlobalCommandStateAction::DONE)

    });

    g.addState(GlobalCommandStateAction::WAITING_FOR_GLOBAL_COMMAND, {
        std::make_pair(GlobalCommandTriggerID::HAS_GLOBAL_COMMAND, GlobalCommandStateAction::DONE)
    });

    return g;
}

bool GlobalCommandStateMachine::transition(const GlobalCommandTrigger &trigger) {
    // Let base class function to graph trasition and set new ego action
    if (!AbstractStateMachine::transition(trigger)) {
        return false;
    }

    // Process trigger (e.g. store stopping distance in state)
    switch (trigger.getID()) {
        case GlobalCommandTriggerID::COMMAND_COMPLETE:
        {
            path_planning::GlobalCommandQueue que = _state.getGlobalCommandQueue();
            que.pop(); 
            _state.setGlobalCommandQueue(que);
            break;
        }
        default:
            return true;
    }

    return true;

}

std::string GlobalCommandStateMachine::getName() const {
    return "GlobalCommandStateMachine";
}
