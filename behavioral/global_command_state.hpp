#pragma once

#include "ros/ros.h"
#include "common/line_segment.hpp"
#include "common/lane.hpp"
#include "global_command.hpp"
#include "common/stopline.hpp"
#include "abstract_state.hpp"
#include "common/polyline.hpp"
#include "common/vehicle_state.hpp"
#include "stop_result.hpp"
#include "ego_state.hpp"

using boost::optional;
using path_planning::Lane;

namespace path_planning { namespace behavioral {

    enum class GlobalCommandStateAction {TURNING_RIGHT, TURNING_LEFT, HEADING_STRAIGHT, STOPPING, WAITING_FOR_GLOBAL_COMMAND, DONE};

    class GlobalCommandState : public AbstractState<GlobalCommandStateAction> {
        public:

            GlobalCommandState(GlobalCommandStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            GlobalCommandState setGlobalCommandQueue(const GlobalCommandQueue &cmds);
            GlobalCommandQueue getGlobalCommandQueue() const;

            GlobalCommandState setVehiclePose(const VehicleState &pose);
            VehicleState& getVehiclePose();

            EgoState::PathResult getPathResult(); 

            GlobalCommandState setPathResult(const EgoState::PathResult &result);

        private:
            GlobalCommandQueue _global_cmd_queue;
            VehicleState _vehicle_pose;
            bool _is_command_change;
            EgoState::PathResult _path_result;            
    };

}}
