#pragma once

#include "abstract_state_machine.hpp"
#include "global_command_state.hpp"
#include "global_command_trigger.hpp"
#include "global_command_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class GlobalCommandStateMachine : public AbstractStateMachine<GlobalCommandState, GlobalCommandTrigger, GlobalCommandStateAction, GlobalCommandTriggerID> {
        public:
            GlobalCommandStateMachine();
            std::string getName() const override;

        private:
            AbstractTransitionGraph<GlobalCommandStateAction, GlobalCommandTriggerID> createTransitionGraph() override;

            bool transition(const GlobalCommandTrigger &trigger) override;
    };

}}