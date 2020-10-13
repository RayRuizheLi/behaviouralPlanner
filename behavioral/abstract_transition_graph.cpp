#include "abstract_transition_graph.hpp"

using namespace path_planning::behavioral;

template <typename StateAction, typename TriggerID>
void AbstractTransitionGraph<StateAction, TriggerID>::addState(StateAction state, std::vector<TransitionIn> transitions) {
    std::vector<TriggerID> stateTriggers;
    for (const TransitionIn &transition : transitions) {
        transition_graph[std::make_pair(state, transition.first)] = transition.second;
        stateTriggers.push_back(transition.first);
    }
    trigger_map[state] = stateTriggers;
}

template <typename StateAction, typename TriggerID>
std::vector<TriggerID> AbstractTransitionGraph<StateAction, TriggerID>::getTriggers(StateAction state) const {
    if (trigger_map.find(state) == trigger_map.end()) {
        return {};
    }
    return trigger_map.at(state);
}

template <typename StateAction, typename TriggerID>
boost::optional<StateAction> AbstractTransitionGraph<StateAction, TriggerID>::getTransition(TransitionOut transition) const {
    boost::optional<StateAction> ret = boost::none;
    if (transition_graph.find(transition) != transition_graph.end()) {
        ret = transition_graph.at(transition);
    }
    return ret;
}

template class AbstractTransitionGraph<TrafficLightStateAction, TrafficLightTriggerID>;
template class AbstractTransitionGraph<EgoStateAction, EgoTriggerID>;
template class AbstractTransitionGraph<StopSignStateAction, StopSignTriggerID>;
template class AbstractTransitionGraph<PedestrianStateAction, PedestrianTriggerID>;
template class AbstractTransitionGraph<ObstacleStateAction, ObstacleTriggerID>;
template class AbstractTransitionGraph<GlobalCommandStateAction, GlobalCommandTriggerID>;
