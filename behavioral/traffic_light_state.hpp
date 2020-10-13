#pragma once

#include "ros/ros.h"
#include "common/line_segment.hpp"
#include "common/traffic_light.hpp"
#include "global_command.hpp"
#include "common/stopline.hpp"
#include "abstract_state.hpp"
#include "common/polyline.hpp"
#include "common/vehicle_state.hpp"
#include "common/lane.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    enum class TrafficLightStateAction {NON_BLOCKING, BLOCKING};

    class TrafficLightState : public AbstractState<TrafficLightStateAction> {
        public:

            TrafficLightState(TrafficLight light, TrafficLightStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            boost::optional<StopLine> getStoppingLine() const;

            TrafficLightState setLightStatus(TrafficLight light);
            TrafficLight getLightStatus() const;

            TrafficLightState setGlobalCommandQueue(GlobalCommandQueue cmds);
            GlobalCommandQueue getGlobalCommandQueue() const;

            TrafficLightState setCurrentLane(const Lane &lane);
            const Lane& getCurrentLane() const;

            TrafficLightState setTrajectoryReference(const geom::Polyline& line);
            const geom::Polyline& getTrajectoryReference() const;

            TrafficLightState setVehiclePose(const VehicleState &pose);
            const VehicleState& getVehiclePose() const;

            void updateStopResult();
            TrafficLightState setStopResult(const StopResult &result);
            const StopResult& getStopResult() const;

        private:
            TrafficLight _light;
            GlobalCommandQueue _globalCommandQueue;
            geom::Polyline _curr_trajectory_reference;
            VehicleState _vehicle_pose;
            Lane _current_lane;
            StopResult _stop_result;
    };

}}