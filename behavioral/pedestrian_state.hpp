#pragma once

#include "abstract_state.hpp"
#include "common/pedestrian.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    enum class PedestrianStateAction {IGNORED, NON_ACTIVE, ACTIVE, PAUSED};

    class PedestrianState : public AbstractState<PedestrianStateAction> {
        public:

            PedestrianState(Pedestrian ped_obj, PedestrianStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            PedestrianState setPedestrianObj(const Pedestrian& ped_obj);
            const Pedestrian& getPedestrianObj() const;

            PedestrianState setTrajectoryReference(const geom::Polyline& line);
            const geom::Polyline& getTrajectoryReference() const;

            PedestrianState setVehiclePose(const VehicleState &pose);
            const VehicleState& getVehiclePose() const;

            void updateStopResult();
            PedestrianState setStopResult(const StopResult &result);
            const StopResult& getStopResult() const;

            PedestrianState setPausedTime(ros::Time time);
            ros::Time getPausedTime() const;

        private:
            Pedestrian _ped_obj;
            VehicleState _vehicle_pose;
            geom::Polyline _trajectory_reference;
            StopResult _stop_result;
            ros::Time _paused_time;
    };

} }