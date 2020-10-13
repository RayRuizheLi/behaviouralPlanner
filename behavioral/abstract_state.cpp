#include "abstract_state.hpp"

// Need these in cpp file to prevent circular include in hpp files
#include "ego_state.hpp"
#include "traffic_light_state.hpp"
#include "stop_sign_state.hpp"
#include "pedestrian_state.hpp"
#include "obstacle_state.hpp"
#include "obstacle_trigger.hpp"
#include "global_command_state.hpp"

using namespace path_planning::behavioral;

template<typename StateAction>
AbstractState<StateAction>::AbstractState(StateAction action): _action{action} {} 

template<typename StateAction>
StateAction AbstractState<StateAction>::getAction() const {
    return _action;
}

template<typename StateAction>
void AbstractState<StateAction>::setAction(StateAction action)  {
    _action = action;
}

template class AbstractState<TrafficLightStateAction>;
template class AbstractState<EgoStateAction>;
template class AbstractState<StopSignStateAction>;
template class AbstractState<PedestrianStateAction>;
template class AbstractState<ObstacleStateAction>;
template class AbstractState<GlobalCommandStateAction>;
