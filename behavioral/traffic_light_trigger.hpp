#pragma once

#include "traffic_light_state.hpp"
#include "abstract_trigger.hpp"
#include "common/traffic_light.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    class TrafficLightTriggerFactory;
    enum class TrafficLightTriggerID {PRESENT_ON_ROUTE, REMOVED};

    class TrafficLightTrigger: public AbstractTrigger<TrafficLightTriggerID, TrafficLightState> {
        protected:
            TrafficLightTrigger(TrafficLightTriggerID id, TrafficLightState state): AbstractTrigger(id, state) {};
    };
    
    /**
     * Condition: When the traffic light is blocking the next global command and the referenceTrajectory goes through light's stopline
     */
    class TrafficLightPresentOnRouteTrigger: public TrafficLightTrigger {
        friend TrafficLightTriggerFactory;
        public:
            bool test() override;

            StopResult stop_result;
        private:
            TrafficLightPresentOnRouteTrigger(TrafficLightState state): TrafficLightTrigger(TrafficLightTriggerID::PRESENT_ON_ROUTE, state) {};
    };

    /**
     * Condition: When the traffic light is not blocking the next global command
     */

    class TrafficLightRemovedTrigger: public TrafficLightTrigger {
        friend TrafficLightTriggerFactory;
        public:
            bool test() override;
        private:
            TrafficLightRemovedTrigger(TrafficLightState state): TrafficLightTrigger(TrafficLightTriggerID::REMOVED, state) {};
    };

}}