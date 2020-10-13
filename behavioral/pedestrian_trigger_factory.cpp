#include "pedestrian_trigger_factory.hpp"

using namespace path_planning::behavioral;

std::shared_ptr<PedestrianTrigger> PedestrianTriggerFactory::enumToType(PedestrianTriggerID id, PedestrianState state) const {
    switch (id) {
        case PedestrianTriggerID::IN_ROI:
            return std::shared_ptr<PedestrianInRoITrigger>(new PedestrianInRoITrigger(state));
        case PedestrianTriggerID::MOVING:
            return std::shared_ptr<PedestrianMovingTrigger>(new PedestrianMovingTrigger(state));
        case PedestrianTriggerID::NOT_MOVING:
            return std::shared_ptr<PedestrianNotMovingTrigger>(new PedestrianNotMovingTrigger(state));
        case PedestrianTriggerID::NOT_IN_ROI:
            return std::shared_ptr<PedestrianNotInRoI>(new PedestrianNotInRoI(state));
        case PedestrianTriggerID::STOPPED:
            return std::shared_ptr<PedestrianStoppedTrigger>(new PedestrianStoppedTrigger(state));
        default:
            throw std::invalid_argument("No class set up for pedestrian trigger");
    }
}