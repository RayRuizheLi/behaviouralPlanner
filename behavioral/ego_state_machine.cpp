#include "ego_state_machine.hpp"
#include "ego_trigger_factory.hpp"

using namespace path_planning::behavioral;

AbstractTransitionGraph<EgoStateAction, EgoTriggerID> EgoStateMachine::createTransitionGraph() {
    AbstractTransitionGraph<EgoStateAction, EgoTriggerID> g;
    g.addState(EgoStateAction::FOLLOWING_LANE, {
        std::make_pair(EgoTriggerID::CHANGE_LANE_LEFT, EgoStateAction::CHANGING_LANE_LEFT), 
        std::make_pair(EgoTriggerID::CHANGE_LANE_RIGHT, EgoStateAction::CHANGING_LANE_RIGHT),
        std::make_pair(EgoTriggerID::STOP_LINE_ON_ROUTE, EgoStateAction::STOPPING),
        std::make_pair(EgoTriggerID::NO_GLOBAL_COMMANDS, EgoStateAction::WAITING_FOR_COMMANDS)
    });

    // TODO: Can we change to stopping state while in Lane Change?
    g.addState(EgoStateAction::CHANGING_LANE_RIGHT, {
        std::make_pair(EgoTriggerID::CHANGE_LANE_LEFT, EgoStateAction::CHANGING_LANE_LEFT), 
        std::make_pair(EgoTriggerID::LANE_STABILIZED, EgoStateAction::FOLLOWING_LANE),
        std::make_pair(EgoTriggerID::NO_GLOBAL_COMMANDS, EgoStateAction::WAITING_FOR_COMMANDS)
    });

    // TODO: Can we change to stopping state while in Lane Change?
    g.addState(EgoStateAction::CHANGING_LANE_LEFT, {
        std::make_pair(EgoTriggerID::CHANGE_LANE_RIGHT, EgoStateAction::CHANGING_LANE_RIGHT), 
        std::make_pair(EgoTriggerID::LANE_STABILIZED, EgoStateAction::FOLLOWING_LANE),
        std::make_pair(EgoTriggerID::NO_GLOBAL_COMMANDS, EgoStateAction::WAITING_FOR_COMMANDS)
    });

    g.addState(EgoStateAction::STOPPING, {
        std::make_pair(EgoTriggerID::NO_STOP_LINE_ON_ROUTE, EgoStateAction::FOLLOWING_LANE),
        std::make_pair(EgoTriggerID::NO_GLOBAL_COMMANDS, EgoStateAction::WAITING_FOR_COMMANDS)
    });

    g.addState(EgoStateAction::WAITING_FOR_COMMANDS, {
        std::make_pair(EgoTriggerID::HAS_GLOBAL_COMMANDS, EgoStateAction::FOLLOWING_LANE) 
    });

    return g;
} 

EgoStateMachine::EgoStateMachine(const ros::Publisher &light_lock_publisher): 
    AbstractStateMachine(
        createTransitionGraph(),
        std::move(std::unique_ptr<EgoTriggerFactory>(new EgoTriggerFactory())),
        EgoState(EgoStateAction::WAITING_FOR_COMMANDS)
    ),
    light_lock_publisher{light_lock_publisher} {
        ROS_INFO_STREAM("[" << getName() << "]\n" << "Created with initial state action: " << _state.serializeAction());
    }

std::string EgoStateMachine::getName() const {
    return "EgoStateMachine";
}


bool EgoStateMachine::transition(const EgoTrigger &trigger) {
    // Let base class function to graph trasition and set new ego action
    if (!AbstractStateMachine::transition(trigger)) {
        return false;
    }

    // Process trigger (e.g. turn on blinker light)
    embedded_msgs::LockLightingRequest msg;
    switch (trigger.getID()) {

        case EgoTriggerID::CHANGE_LANE_LEFT:
            msg.lamp = embedded_msgs::LockLightingRequest::LEFT_LAMP;
            light_lock_publisher.publish(msg);
            break;
        case EgoTriggerID::CHANGE_LANE_RIGHT:
            msg.lamp = embedded_msgs::LockLightingRequest::RIGHT_LAMP;
            light_lock_publisher.publish(msg);
            break;
        case EgoTriggerID::LANE_STABILIZED:
            msg.lamp = embedded_msgs::LockLightingRequest::NO_LAMP;
            light_lock_publisher.publish(msg);
            break;
        default:
            return true;

        // TODO: Process waiting for re-route trigger as so:
        // catch (path_planning::global_reroute& e) {
        // currentDesired = {e.current, e.current};
        // global_reroute_publisher->publish(
        //   ros_msgs::generateGlobalCommandMessage(global_interface->getGlobalCommandQueue().front()));
        // }
    }    

    return true;
}

void EgoStateMachine::cycle(){
    _state.updatePath();

    // Let base class handle doing graph cycle and calling transition
    AbstractStateMachine::cycle();
}
