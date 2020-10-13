#pragma once

#include "abstract_state.hpp"
#include "common/obstacle.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    enum class ObstacleStateAction {IGNORED, MOVING, STOPPED};

    class ObstacleState : public AbstractState<ObstacleStateAction> {
        public:

            ObstacleState(Obstacle obj, ObstacleStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            ObstacleState setObstacle(const Obstacle& obj);
            const Obstacle& getObstacle() const;

            ObstacleState setTrajectoryReference(const geom::Polyline& line);
            const geom::Polyline& getTrajectoryReference() const;

            ObstacleState setVehiclePose(const VehicleState &pose);
            const VehicleState& getVehiclePose() const;

            void updateStopResult();
            ObstacleState setStopResult(const StopResult &result);
            const StopResult& getStopResult() const;

        private:
            Obstacle _obj;
            VehicleState _vehicle_pose;
            geom::Polyline _trajectory_reference;
            StopResult _stop_result;
    };

} }