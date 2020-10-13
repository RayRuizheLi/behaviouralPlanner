#pragma once

#include "abstract_state_machine.hpp"
#include "obstacle_state.hpp"
#include "obstacle_trigger.hpp"
#include "obstacle_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class ObstacleStateMachine : public AbstractStateMachine<ObstacleState, ObstacleTrigger, ObstacleStateAction, ObstacleTriggerID> {
        public:
            ObstacleStateMachine(Obstacle initial_obj);

            void cycle() override;

            std::string getName() const override;
            
        private:
            AbstractTransitionGraph<ObstacleStateAction, ObstacleTriggerID> createTransitionGraph() override;

            bool transition(const ObstacleTrigger &trigger) override;
    };

}}