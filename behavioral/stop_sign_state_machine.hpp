#pragma once

#include "abstract_state_machine.hpp"
#include "stop_sign_state.hpp"
#include "stop_sign_trigger.hpp"
#include "stop_sign_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class StopSignStateMachine : public AbstractStateMachine<StopSignState, StopSignTrigger, StopSignStateAction, StopSignTriggerID> {
        public:
            StopSignStateMachine(StopSign initial_sign);

            void cycle() override;

            std::string getName() const override;
            
        private:
            AbstractTransitionGraph<StopSignStateAction, StopSignTriggerID> createTransitionGraph() override;

            bool transition(const StopSignTrigger &trigger) override;
    };

}}