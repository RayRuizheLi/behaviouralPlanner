#pragma once

#include "abstract_trigger_factory.hpp"
#include "obstacle_state.hpp"
#include "obstacle_trigger.hpp"

namespace path_planning { namespace behavioral {

    class ObstacleTriggerFactory : public AbstractTriggerFactory<ObstacleTrigger, ObstacleState, ObstacleTriggerID> {
        private:
            std::shared_ptr<ObstacleTrigger> enumToType(ObstacleTriggerID id, ObstacleState state) const override;
    };

}}