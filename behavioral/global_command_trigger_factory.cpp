#include "global_command_trigger_factory.hpp"

using namespace path_planning::behavioral;

std::shared_ptr<GlobalCommandTrigger> GlobalCommandTriggerFactory::enumToType(GlobalCommandTriggerID id, GlobalCommandState state) const {
    switch (id) {
        case GlobalCommandTriggerID::TURN_RIGHT:
            return std::shared_ptr<TriggerTurnRight>(new TriggerTurnRight(state));
        case GlobalCommandTriggerID::TURN_LEFT:
            return std::shared_ptr<TriggerTurnLeft>(new TriggerTurnLeft(state));
        case GlobalCommandTriggerID::STOP:
            return std::shared_ptr<TriggerStop>(new TriggerStop(state));
        case GlobalCommandTriggerID::HEAD_STRAIGHT:
            return std::shared_ptr<TriggerHeadStraight>(new TriggerHeadStraight(state));
        case GlobalCommandTriggerID::NO_GLOBAL_COMMAND:
            return std::shared_ptr<TriggerNoGlobalCommand>(new TriggerNoGlobalCommand(state));
        case GlobalCommandTriggerID::HAS_GLOBAL_COMMAND:
            return std::shared_ptr<TriggerHasGlobalCommand>(new TriggerHasGlobalCommand(state));
        case GlobalCommandTriggerID::COMMAND_COMPLETE:
            return std::shared_ptr<TriggerGlobalCommandComplete>(new TriggerGlobalCommandComplete(state));
        default:
            throw std::invalid_argument("No class set up for global command trigger");
    }
}