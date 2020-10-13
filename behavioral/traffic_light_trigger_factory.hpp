#pragma once

#include "abstract_trigger_factory.hpp"
#include "traffic_light_state.hpp"
#include "traffic_light_trigger.hpp"

namespace path_planning { namespace behavioral {

    class TrafficLightTriggerFactory : public AbstractTriggerFactory<TrafficLightTrigger, TrafficLightState, TrafficLightTriggerID> {
        private:
            std::shared_ptr<TrafficLightTrigger> enumToType(TrafficLightTriggerID id, TrafficLightState state) const override;
    };

}}