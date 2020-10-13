#include "abstract_trigger.hpp"

// Need these in cpp file to prevent circular include in hpp files
#include "traffic_light_state.hpp"
#include "traffic_light_trigger.hpp"
#include "ego_state.hpp"
#include "ego_trigger.hpp"
#include "stop_sign_state.hpp"
#include "stop_sign_trigger.hpp"
#include "pedestrian_state.hpp"
#include "pedestrian_trigger.hpp"
#include "obstacle_state.hpp"
#include "obstacle_trigger.hpp"
#include "global_command_trigger.hpp"

using namespace path_planning::behavioral;

template<typename TriggerID, typename State>
AbstractTrigger<TriggerID, State>::AbstractTrigger(TriggerID id, State state): _state{state}, _id{id} {} 

template<typename TriggerID, typename State>
TriggerID AbstractTrigger<TriggerID, State>::getID() const {
    return _id;
}

template class AbstractTrigger<TrafficLightTriggerID, TrafficLightState>;
template class AbstractTrigger<EgoTriggerID, EgoState>;
template class AbstractTrigger<StopSignTriggerID, StopSignState>;
template class AbstractTrigger<PedestrianTriggerID, PedestrianState>;
template class AbstractTrigger<ObstacleTriggerID, ObstacleState>;
template class AbstractTrigger<GlobalCommandTriggerID, GlobalCommandState>;
