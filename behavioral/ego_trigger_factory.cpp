#include "ego_trigger_factory.hpp"

using namespace path_planning::behavioral;

// TODO: Implement a more sophisticated pattern, e.g. https://codereview.stackexchange.com/questions/166634/instantiating-a-c-class-based-on-an-enum-value
std::shared_ptr<EgoTrigger> EgoTriggerFactory::enumToType(EgoTriggerID id, EgoState ego_state) const {
    switch(id) {
        case EgoTriggerID::CHANGE_LANE_LEFT:
            return std::shared_ptr<TriggerLeftLaneChange>(new TriggerLeftLaneChange(ego_state));
        case EgoTriggerID::CHANGE_LANE_RIGHT:
            return std::shared_ptr<TriggerRightLaneChange>(new TriggerRightLaneChange(ego_state));
        case EgoTriggerID::LANE_STABILIZED:
            return std::shared_ptr<TriggerLaneStabilized>(new TriggerLaneStabilized(ego_state)); 
        case EgoTriggerID::STOP_LINE_ON_ROUTE:
            return std::shared_ptr<TriggerStopLineOnRoute>(new TriggerStopLineOnRoute(ego_state)); 
        case EgoTriggerID::NO_STOP_LINE_ON_ROUTE:
            return std::shared_ptr<TriggerNoStopLineOnRoute>(new TriggerNoStopLineOnRoute(ego_state)); 
        case EgoTriggerID::NO_GLOBAL_COMMANDS:
            return std::shared_ptr<TriggerNoGlobalCommands>(new TriggerNoGlobalCommands(ego_state));
        case EgoTriggerID::HAS_GLOBAL_COMMANDS:
            return std::shared_ptr<TriggerHasGlobalCommands>(new TriggerHasGlobalCommands(ego_state));
        default:
            throw std::invalid_argument("No class set up for trigger in EgoTriggerFactory::enumToType");
    }
}