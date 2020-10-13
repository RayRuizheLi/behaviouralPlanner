#pragma once

#include <utility>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <memory>
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

namespace path_planning { namespace behavioral {

    /** 
     * StateAction is an enum class, defined in the header file of an AbstractState dervied class
     * TriggerID is an enum class, defined in the header file of an AbstractTrigger derived class
     */
    template <typename StateAction, typename TriggerID>
    class AbstractTransitionGraph {
        public:
            typedef std::pair<StateAction, TriggerID> TransitionOut;
            typedef std::pair<TriggerID, StateAction> TransitionIn;

            void addState(StateAction state, std::vector<TransitionIn> transitions);
            std::vector<TriggerID> getTriggers(StateAction state) const;
            boost::optional<StateAction> getTransition(TransitionOut transition) const;

        private:
            std::map<StateAction, std::vector<TriggerID>> trigger_map;
            std::map<TransitionOut, StateAction> transition_graph;

    };

}}