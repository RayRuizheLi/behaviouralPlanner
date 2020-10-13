#pragma once

#include <deque>
#include <stack>
#include "global_command.hpp"
#include "common/vehicle_state.hpp"
#include "common/lane.hpp"
#include "common/point2d.hpp"
#include "common/polyline.hpp"
#include "path_planning_msgs/EgoVehicleState.h"
#include "HDM_interface.hpp"
#include "common/stopline.hpp"
#include "abstract_state.hpp"
#include "stop_result.hpp"
#include <exception>

using boost::optional;
using path_planning::Lane;

namespace path_planning { namespace behavioral {

    enum class EgoStateAction {FOLLOWING_LANE, CHANGING_LANE_LEFT, CHANGING_LANE_RIGHT, STOPPING, WAITING_FOR_COMMANDS};

    class EgoState : public AbstractState<EgoStateAction> {
        public:
            EgoState(EgoStateAction action);

            path_planning_msgs::EgoVehicleState getStateMsg() const;
            std::string serializeAction() const override;
            std::string serialize() const override;

            EgoState addLatDev(double dev);
            EgoState addDesiredLatDev(double dev);
            double getLatestLatDev() const;
            double getLatestDesiredLatDev() const;
            std::deque<double> getLatDevHist() const;
            std::deque<double> getDesiredLatDevHist() const;

            VehicleState getPose() const;
            EgoState setPose(VehicleState pose);

            EgoState setWorldStopResult(const StopResult& result);
            const StopResult& getWorldStopResult() const;

            GlobalCommand getGlobalCommand(); 

            struct PathResult {
                public:
                    enum RESULT_TYPE {NONE, UNKNOWN_FAILURE, SEARCH_FAILURE, NO_GLOBAL_COMMAND, HAS_PATH};
                    bool hasPath() const;
                    optional<int> getLastLaneletID() const;
                    optional<std::deque<Lane>> getPath() const;
                    optional<path_planning::geom::Polyline> getFrenetReferenceLine() const;
                    void setPath(std::deque<Lane> path);
                    void setNoPath(RESULT_TYPE result, std::string what="");
                    void prunePath(VehicleState pose);

                    std::string serializeResult() const;
                    void printPath() const;
                private:
                    RESULT_TYPE _result;
                    std::deque<Lane> _path;
                    std::string _what_failure;
                    bool shouldPrunePath(VehicleState pose) const;
            };

            PathResult getPathResult() const;

            void updatePath();
            EgoState setGlobalCommand(const GlobalCommand &cmd);

        private:
            const static unsigned int HISTORY_SIZE = 10;

            std::deque<double> _lat_dev_hist;
            std::deque<double> _desired_lat_dev_hist;

            VehicleState _pose;

            LaneTopology _topology;
            PathResult _path_result;
            HDMInterface _hdm_interface;

            StopResult _world_stop_result;

            optional<std::deque<Lane>> tryFindLaneletPath(
                Lane startLanelet, 
                GlobalCommand goalCommand
            );
            Lane queryLane(int lane_id);
            double distanceToPath() const;
            GlobalCommand _cmd;
    };
    
} }
