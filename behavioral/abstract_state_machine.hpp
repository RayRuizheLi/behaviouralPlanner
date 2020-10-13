#pragma once

#include "abstract_transition_graph.hpp"
#include "abstract_trigger_factory.hpp"
#include "traffic_light_state.hpp"
#include "traffic_light_trigger.hpp"
#include "ego_state.hpp"
#include "ego_trigger.hpp"
#include "stop_sign_state.hpp"
#include "stop_sign_trigger.hpp"
#include "pedestrian_state.hpp"
#include "pedestrian_trigger.hpp"
#include "obstacle_state.hpp"
#include "obstacle_trigger.hpp"
#include "global_command_state.hpp"
#include "global_command_trigger.hpp"
#include <memory>

namespace path_planning { namespace behavioral {

    /** 
     * State is a class derived from AbstractState
     * Trigger is a class derived from Abstract Trigger
     * StateAction is an enum class defined in the header file of State
     * TriggerID is an enum class defined in the header file of Trigger
     */
    template <typename State, typename Trigger, typename StateAction, typename TriggerID>
    class AbstractStateMachine {
        public:
            AbstractStateMachine(AbstractTransitionGraph<StateAction, TriggerID> transition_graph, 
                std::unique_ptr<AbstractTriggerFactory<Trigger, State, TriggerID>> _trigger_factory,
                State _state);

            State getState() const;
            void setState(State state);
            virtual void cycle();
            virtual std::string getName() const = 0;

        protected:
            State _state;

            virtual bool transition(const Trigger &trigger);

        private:
            virtual AbstractTransitionGraph<StateAction, TriggerID> createTransitionGraph() = 0;

            const AbstractTransitionGraph<StateAction, TriggerID> _transition_graph;
            const std::unique_ptr<AbstractTriggerFactory<Trigger, State, TriggerID>> _trigger_factory;

    };

} }