#include "obstacle_trigger_factory.hpp"

using namespace path_planning::behavioral;

std::shared_ptr<ObstacleTrigger> ObstacleTriggerFactory::enumToType(ObstacleTriggerID id, ObstacleState state) const {
    switch (id) {
        case ObstacleTriggerID::IN_ROI:
            return std::shared_ptr<ObstacleInRoITrigger>(new ObstacleInRoITrigger(state));
        case ObstacleTriggerID::MOVING:
            return std::shared_ptr<ObstacleMovingTrigger>(new ObstacleMovingTrigger(state));
        case ObstacleTriggerID::NOT_IN_ROI:
            return std::shared_ptr<ObstacleNotInRoI>(new ObstacleNotInRoI(state));
        case ObstacleTriggerID::STOPPED:
            return std::shared_ptr<ObstacleStoppedTrigger>(new ObstacleStoppedTrigger(state));
        default:
            throw std::invalid_argument("No class set up for Obstacle trigger");
    }
}