#pragma one

#include <vector>
#include <map>
#include <memory>
#include "common/line_segment.hpp"
#include "traffic_light_state_machine.hpp"
#include "common/stopline.hpp"
#include "common/polyline.hpp"
#include "common/environment.hpp"
#include "global_command.hpp"
#include "common/vehicle_state.hpp"
#include "stop_result.hpp"
#include "stop_sign_state_machine.hpp"
#include "pedestrian_state_machine.hpp"
#include "common/lane.hpp"
#include "obstacle_state_machine.hpp"
#include <unordered_set>
#include <utility>

namespace path_planning { namespace behavioral {

    class WorldStateManager {

        public:

            void updateEnvironment(const path_planning::Environment &env);

            void updateGlobalCommandQueue(path_planning::GlobalCommandQueue cmds);

            void updateTrajectoryReference(const geom::Polyline& line);

            void updateCurrentLane(const Lane& lane);

            void updateVehiclePose(const VehicleState& pose);

            void cycleMachines();

            std::vector<path_planning::StopLine> getStopLines() const;

            StopResult getStopResult() const;

        private:
            std::map<int, std::shared_ptr<TrafficLightStateMachine>> _traffic_lights;
            std::map<int, std::shared_ptr<StopSignStateMachine>> _stop_signs;
            std::map<int, std::shared_ptr<PedestrianStateMachine>> _pedestrians;
            std::map<int, std::shared_ptr<ObstacleStateMachine>> _obstacles;
    };

}}