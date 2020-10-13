#pragma once

#include "abstract_state_machine.hpp"
#include "pedestrian_state.hpp"
#include "pedestrian_trigger.hpp"
#include "pedestrian_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class PedestrianStateMachine : public AbstractStateMachine<PedestrianState, PedestrianTrigger, PedestrianStateAction, PedestrianTriggerID> {
        public:
            PedestrianStateMachine(Pedestrian initial_sign);

            void cycle() override;

            std::string getName() const override;
            
        private:
            AbstractTransitionGraph<PedestrianStateAction, PedestrianTriggerID> createTransitionGraph() override;

            bool transition(const PedestrianTrigger &trigger) override;
    };

}}