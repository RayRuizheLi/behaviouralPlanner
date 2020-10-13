#pragma once

#include "abstract_trigger.hpp"
#include "obstacle_state.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    class ObstacleTriggerFactory;

    enum class ObstacleTriggerID {IN_ROI, NOT_IN_ROI, MOVING, STOPPED};

    class ObstacleTrigger : public AbstractTrigger<ObstacleTriggerID, ObstacleState> {
        protected:
            ObstacleTrigger(ObstacleTriggerID id, ObstacleState state): AbstractTrigger(id, state) {};
    };

    /** 
     * Condition: When the Obstacle is near the reference line
     */

    class ObstacleInRoITrigger: public ObstacleTrigger {
        friend ObstacleTriggerFactory;
        public:
            bool test() override;

            StopResult stop_result;

        private:
            ObstacleInRoITrigger(ObstacleState state): ObstacleTrigger(ObstacleTriggerID::IN_ROI, state) {};
    };


    /** 
     * Condition: When the magnitute of the Obstacle's velocity vector is greater than config('Obstacle_MOVEMENT_EPSILON')
     */

    class ObstacleMovingTrigger: public ObstacleTrigger {
        friend ObstacleTriggerFactory;
        public:
            bool test() override;

        private:
            ObstacleMovingTrigger(ObstacleState state): ObstacleTrigger(ObstacleTriggerID::MOVING, state) {};
    };

    /** 
     * Condition: When the Obstacle is not near the reference line
     */

    class ObstacleNotInRoI: public ObstacleTrigger {
        friend ObstacleTriggerFactory;
        public:
            bool test() override;

        private:
            ObstacleNotInRoI(ObstacleState state): ObstacleTrigger(ObstacleTriggerID::NOT_IN_ROI, state) {};
    };

    /** 
     * Condition: When the magnitute of the Obstacle's velocity vector is less than config('Obstacle_MOVEMENT_EPSILON')
     */

    class ObstacleStoppedTrigger: public ObstacleTrigger {
        friend ObstacleTriggerFactory;
        public:
            bool test() override;

        private:
            ObstacleStoppedTrigger(ObstacleState state): ObstacleTrigger(ObstacleTriggerID::STOPPED, state) {};
    };

}}