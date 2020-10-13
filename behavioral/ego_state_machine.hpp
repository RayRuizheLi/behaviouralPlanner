#pragma once

#include <vector>
#include <memory>
#include <ros/ros.h>
#include <path_planning/DynamicConfig.h>
#include <embedded_msgs/LockLightingRequest.h>
#include "ego_state.hpp"
#include "ego_trigger.hpp"
#include "abstract_transition_graph.hpp"
#include "abstract_state_machine.hpp"
#include "global_interface.hpp"
#include "global_command.hpp"

namespace path_planning { namespace behavioral {

    class EgoStateMachine : public AbstractStateMachine<EgoState, EgoTrigger, EgoStateAction, EgoTriggerID> {
        public:
            EgoStateMachine(
                const ros::Publisher &light_lock_publisher
            );

            void cycle() override;
            std::string getName() const override;
            
        private:
            AbstractTransitionGraph<EgoStateAction, EgoTriggerID> createTransitionGraph() override;
            const ros::Publisher &light_lock_publisher;

            bool transition(const EgoTrigger &trigger) override;
    };

}}