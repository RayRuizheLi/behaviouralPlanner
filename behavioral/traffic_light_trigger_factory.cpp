#include "traffic_light_trigger_factory.hpp"

using namespace path_planning::behavioral;

std::shared_ptr<TrafficLightTrigger> TrafficLightTriggerFactory::enumToType(TrafficLightTriggerID id, TrafficLightState state) const {
    switch (id) {
        case TrafficLightTriggerID::PRESENT_ON_ROUTE:
            return std::shared_ptr<TrafficLightPresentOnRouteTrigger>(new TrafficLightPresentOnRouteTrigger(state));
        case TrafficLightTriggerID::REMOVED:
            return std::shared_ptr<TrafficLightRemovedTrigger>(new TrafficLightRemovedTrigger(state));
        default:
            throw std::invalid_argument("No class set up for traffic light trigger");
    }
}