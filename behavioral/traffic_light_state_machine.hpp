#pragma once

#include "abstract_state_machine.hpp"
#include "traffic_light_state.hpp"
#include "traffic_light_trigger.hpp"
#include "traffic_light_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class TrafficLightStateMachine : public AbstractStateMachine<TrafficLightState, TrafficLightTrigger, TrafficLightStateAction, TrafficLightTriggerID> {
        public:
            TrafficLightStateMachine(path_planning::TrafficLight initial_light);

            void cycle() override;

            std::string getName() const override;
            
        private:
            AbstractTransitionGraph<TrafficLightStateAction, TrafficLightTriggerID> createTransitionGraph() override;

            bool transition(const TrafficLightTrigger &trigger) override;
    };

}}