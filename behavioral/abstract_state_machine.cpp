#include "abstract_state_machine.hpp"

using namespace path_planning::behavioral;

template <typename State, typename Trigger, typename StateAction, typename TriggerID>
AbstractStateMachine<State, Trigger, StateAction, TriggerID>::AbstractStateMachine(
    AbstractTransitionGraph<StateAction, TriggerID> transition_graph, 
    std::unique_ptr<AbstractTriggerFactory<Trigger, State, TriggerID>> _trigger_factory,
    State state): 
    _transition_graph{transition_graph}, _trigger_factory{std::move(_trigger_factory)}, _state{state} {}

template <typename State, typename Trigger, typename StateAction, typename TriggerID>
State AbstractStateMachine<State, Trigger, StateAction, TriggerID>::getState() const {
    return _state;
}

template <typename State, typename Trigger, typename StateAction, typename TriggerID>
void AbstractStateMachine<State, Trigger, StateAction, TriggerID>::setState(State state) {
    _state = state;
}

template <typename State, typename Trigger, typename StateAction, typename TriggerID>
bool AbstractStateMachine<State, Trigger, StateAction, TriggerID>::transition(const Trigger &trigger) {
    typename AbstractTransitionGraph<StateAction, TriggerID>::TransitionOut t = std::make_pair(_state.getAction(), trigger.getID());

    // Check if action transition permitted based on trigger
    if (!_transition_graph.getTransition(t)) {
        return false;
    }

    std::string prevAction = _state.serializeAction();

    _state.setAction(_transition_graph.getTransition(t).get());

    ROS_INFO_STREAM("[" << getName() << "]\n" << prevAction << " -> " << _state.serializeAction());
    ROS_INFO_STREAM_COND(!_state.serialize().empty(), "[" << getName() << "]\n" << "Serialized State:\n" << _state.serialize());

    return true;
}

template <typename State, typename Trigger, typename StateAction, typename TriggerID>
void AbstractStateMachine<State, Trigger, StateAction, TriggerID>::cycle() {
    std::vector<TriggerID> possibleTriggers = _transition_graph.getTriggers(_state.getAction());
    std::vector<std::shared_ptr<Trigger>> triggers;

    for (TriggerID possibleTrigger : possibleTriggers) {
        boost::optional<std::shared_ptr<Trigger>> validTriggerOrNone = _trigger_factory->tryCreateTrigger(possibleTrigger, _state);
        if (validTriggerOrNone) {
            triggers.push_back(validTriggerOrNone.get());
        }
    }

    if (triggers.size() < 1) {
        return;
    }

    // TODO: Resolve multiple valid triggers using priority or something

    // Follow the edge to the next state, acting on any side effects (e.g. turning any blinker)
    transition(*triggers[0]);
}

template class AbstractStateMachine<TrafficLightState, TrafficLightTrigger, TrafficLightStateAction, TrafficLightTriggerID>;
template class AbstractStateMachine<EgoState, EgoTrigger, EgoStateAction, EgoTriggerID>;
template class AbstractStateMachine<StopSignState, StopSignTrigger, StopSignStateAction, StopSignTriggerID>;
template class AbstractStateMachine<PedestrianState, PedestrianTrigger, PedestrianStateAction, PedestrianTriggerID>;
template class AbstractStateMachine<ObstacleState, ObstacleTrigger, ObstacleStateAction, ObstacleTriggerID>;
template class AbstractStateMachine<GlobalCommandState, GlobalCommandTrigger, GlobalCommandStateAction, GlobalCommandTriggerID>;
