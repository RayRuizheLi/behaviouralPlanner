#include "stop_sign_trigger_factory.hpp"

using namespace path_planning::behavioral;

std::shared_ptr<StopSignTrigger> StopSignTriggerFactory::enumToType(StopSignTriggerID id, StopSignState state) const {
    switch (id) {
        case StopSignTriggerID::PRESENT_ON_ROUTE:
            return std::shared_ptr<StopSignPresentOnRouteTrigger>(new StopSignPresentOnRouteTrigger(state));
        case StopSignTriggerID::ACTIVATED:
            return std::shared_ptr<StopSignActivatedTrigger>(new StopSignActivatedTrigger(state));
        case StopSignTriggerID::DEACTIVATED:
            return std::shared_ptr<StopSignDeactivatedTrigger>(new StopSignDeactivatedTrigger(state));
        case StopSignTriggerID::EXPIRED:
            return std::shared_ptr<StopSignExpiredTrigger>(new StopSignExpiredTrigger(state));
        default:
            throw std::invalid_argument("No class set up for stop sign trigger");
    }
}