#pragma once

#include "abstract_state.hpp"
#include "common/stop_sign.hpp"
#include "stop_result.hpp"
#include "common/stopline.hpp"
#include "ros/ros.h"

namespace path_planning { namespace behavioral {

    enum class StopSignStateAction {NON_BLOCKING, BLOCKING, ACTIVE_BLOCKING, INACTIVE};

    class StopSignState : public AbstractState<StopSignStateAction> {
        public:

            StopSignState(StopSign sign, StopSignStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            StopSignState setSign(const StopSign& sign);
            const StopSign& getSign() const;

            StopSignState setTrajectoryReference(const geom::Polyline& line);
            const geom::Polyline& getTrajectoryReference() const;

            StopSignState setVehiclePose(const VehicleState &pose);
            const VehicleState& getVehiclePose() const;

            void updateStopResult();
            StopSignState setStopResult(const StopResult &result);
            const StopResult& getStopResult() const;

            StopSignState setActivatedTime(ros::Time time);
            ros::Time getActivatedTime() const;

            boost::optional<StopLine> getStopLine() const;

        private:
            StopSign _sign;
            VehicleState _vehicle_pose;
            geom::Polyline _trajectory_reference;
            StopResult _stop_result;
            ros::Time _activated_time;
    };

} }