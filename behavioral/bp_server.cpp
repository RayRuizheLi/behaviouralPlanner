#include "ros/ros.h"
#include "path_planning_msgs/FrenetPath.h"
#include "path_planning_msgs/Environment.h"
#include "common_msgs/StopLineList.h"
#include "ego_state_machine.hpp"
#include "ego_trigger_factory.hpp"
#include "global_command_state_machine.hpp"
#include "global_command_state.hpp"
#include "global_command.hpp"
#include <mutex>
#include <memory>
#include <dynamic_reconfigure/client.h>
#include "HDM_interface.hpp"
#include "global_interface.hpp"
#include "common/ros_msgs.hpp"
#include <DynamicNode/parameter_config.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <path_planning_msgs/GlobalCommandList.h>
#include "common/vehicle_state.hpp"
#include "path_planning/GetBehavior.h"
#include "world_state_manager.hpp"

using namespace path_planning_msgs;
using namespace path_planning;
using namespace path_planning::behavioral;

std::unique_ptr<EgoStateMachine> ego_machine;
std::unique_ptr<GlobalCommandStateMachine> global_command_machine;
std::mutex machine_mutex;

std::mutex world_mutex;
std::unique_ptr<WorldStateManager> world_manager;

decision::DynamicConfig Dynamic_Config;

tf::StampedTransform tf_map_odom;
tf::TransformListener* tf_listener_ptr;

ros::Publisher* stop_line_pub;

// ros::Publisher* global_reroute_publisher;

void odom_callback(nav_msgs::Odometry odom) {
    const std::lock_guard<std::mutex> ego_lock(machine_mutex);
    const std::lock_guard<std::mutex> world_lock(world_mutex);

    ego_machine->setState(
        ego_machine->getState()
        .setPose(ros_msgs::stateFromOdom(odom))
    );
    global_command_machine->setState(
        global_command_machine->getState()
        .setVehiclePose(ros_msgs::stateFromOdom(odom))
    );
    world_manager->updateVehiclePose(ros_msgs::stateFromOdom(odom));
}

void lookup_transforms(){
    try{
        tf_listener_ptr->waitForTransform("odom","map", ros::Time(0), ros::Duration(1));
        tf_listener_ptr->lookupTransform("odom", "map", ros::Time(0), tf_map_odom);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
}

void global_command_callback(const path_planning_msgs::GlobalCommandList& msg) {
    const std::lock_guard<std::mutex> lock(machine_mutex);
    path_planning::GlobalCommandQueue q;

    if (tf_map_odom.frame_id_ != "odom"){
        lookup_transforms();
    }

    for (auto& cmd : msg.cmds){
        auto globalCommand = ros_msgs::generateGlobalCommand(cmd);
        // Transform from map frame to odom frame
        globalCommand.point.x += tf_map_odom.getOrigin()[0];
        globalCommand.point.y += tf_map_odom.getOrigin()[1];

        q.push(globalCommand);
    }

    // Clear residual path from previous global command
    ego_machine->getState().getPathResult().setNoPath(EgoState::PathResult::NONE);

    global_command_machine->setState(
        global_command_machine->getState()
        .setGlobalCommandQueue(q)
    );
}

  void goal_pose_callback(const geometry_msgs::PoseStamped& goal_pose) {
    // TODO: add hdm interface to get link id based on coordinates
    // heading shouldn't be necessary since each lane has a direction
    HDMInterface hdm_interface;
    auto target_lane = hdm_interface.getCurrentLane(path_planning::VehicleState(geom::Point2d(goal_pose.pose.position.x, goal_pose.pose.position.y), 0));
    path_planning_msgs::GlobalCommand cmd;
    cmd.x = goal_pose.pose.position.x;
    cmd.y = goal_pose.pose.position.y;
    cmd.instruction = path_planning_msgs::GlobalCommand::STOP;
    cmd.link_id.push_back(target_lane.id);

    path_planning_msgs::GlobalCommandList msg;
    msg.cmds.push_back(cmd);

    global_command_callback(msg);
  }

void optimal_path_callback(const path_planning_msgs::FrenetPath &path) {
    const std::lock_guard<std::mutex> lock(machine_mutex);
    ego_machine->setState(
        ego_machine->getState()
        .addLatDev(path.initial.d)
        .addDesiredLatDev(path.lateral_deviation)
    );
}

void config_callback(const decision::DynamicConfig& config) {
    const std::lock_guard<std::mutex> lock(machine_mutex);
    Dynamic_Config = config;
}

boost::optional<std::pair<common_msgs::HDMLaneList, std::vector<geometry_msgs::Point>>> try_get_route_msg() {
    EgoState::PathResult pathResult = ego_machine->getState().getPathResult();
    if (pathResult.hasPath()) {
        std::deque<Lane> path = pathResult.getPath().get();
        std::vector<Lane> vpath = {path.begin(), path.end()};
        return std::make_pair(ros_msgs::generateHDMLaneList(vpath), 
            ros_msgs::generatePolylineMsg(pathResult.getFrenetReferenceLine().get()));
    }
    else {
        return boost::none;
    }
}

bool get_behavior(GetBehavior::Request &req, GetBehavior::Response &res) {
    const std::lock_guard<std::mutex> lock(machine_mutex);
    auto route_and_line = try_get_route_msg();
    if (route_and_line) {
        res.lanelet_route = route_and_line.get().first;
        res.frenet_reference_line = route_and_line.get().second;
    }
    if (ego_machine->getState().getAction() == EgoStateAction::STOPPING) {
        auto stopResult = ego_machine->getState().getWorldStopResult();
        res.stopping_point = ros_msgs::generatePoint(stopResult.getStopPoint());
        res.stopping_distance = stopResult.getDistance();
    }
    res.ego_state = ego_machine->getState().getStateMsg();

    return true;
}

void environment_callback(const path_planning_msgs::Environment &env) {
    const std::lock_guard<std::mutex> lock(world_mutex);
    world_manager->updateEnvironment(ros_msgs::generateEnvironment(env));
}

void timer_callback(const ros::TimerEvent&) {
    const std::lock_guard<std::mutex> machine_lock(machine_mutex);
    const std::lock_guard<std::mutex> world_lock(world_mutex);

    // Update global command state
    global_command_machine->setState(global_command_machine->getState().setPathResult(ego_machine->getState().getPathResult()));
    global_command_machine->cycle();

    // Update ego state 
    // Only update the command in ego_state if global command queue is not empty 
    if(global_command_machine->getState().getAction() == GlobalCommandStateAction::DONE || global_command_machine->getState().getAction() == GlobalCommandStateAction::WAITING_FOR_GLOBAL_COMMAND)
        ego_machine->setState(ego_machine->getState().setGlobalCommand(path_planning::GlobalCommand()));
    else
        ego_machine->setState(ego_machine->getState().setGlobalCommand(global_command_machine->getState().getGlobalCommandQueue().front()));  

    ego_machine->cycle();

    // Update world state
    world_manager->updateGlobalCommandQueue(global_command_machine->getState().getGlobalCommandQueue());
    if (ego_machine->getState().getPathResult().hasPath()) 
        world_manager->updateTrajectoryReference(ego_machine->getState().getPathResult().getFrenetReferenceLine().get());
    world_manager->cycleMachines();

    ego_machine->setState(ego_machine->getState().setWorldStopResult(world_manager->getStopResult()));
    // add the other source of stop result. One from the world (stop sign, etc) one from the global command (destination)
    //  we need to take the mininum index of the two (whichever one is sooner.)

    // //Update ego state again with world stop line info
    // if(global_command_machine->getState().getAction() == GlobalCommandStateAction::DONE || global_command_machine->getState().getAction() == GlobalCommandStateAction::WAITING_FOR_GLOBAL_COMMAND)
    //     ego_machine->setState(ego_machine->getState().setGlobalCommand(path_planning::GlobalCommand()));
    // else
    //     ego_machine->setState(ego_machine->getState().setGlobalCommand(global_command_machine->getState().getGlobalCommandQueue().front()));  
        
    ego_machine->cycle();
    
    stop_line_pub->publish(ros_msgs::generateStopLineList(world_manager->getStopLines()));

    ROS_DEBUG_STREAM_THROTTLE(1, "[" << ego_machine->getName() << "]\n" << "Serialized EgoState:\n" << ego_machine->getState().serialize());
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::init(argc, argv, "bp_server");
    ros::NodeHandle n;

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;


    ros::Publisher light_lock_publisher = n.advertise<embedded_msgs::LockLightingRequest>("/lock_lighting_request", 1);
    ros::Subscriber frenet_optimal_path_sub = n.subscribe("/path_planning/frenet_optimal_path", 1, optimal_path_callback);
    ros::Subscriber global_instruction_subscriber = n.subscribe("/path_planning/global_command", 1, global_command_callback);
    ros::Subscriber goal_pose_subscriber = n.subscribe("/path_planning/goal_pose", 1, goal_pose_callback);
    ros::Subscriber odom_subscriber = n.subscribe("/navsat/odom", 1, odom_callback);
    ros::Subscriber env_subscriber = n.subscribe("/path_planning/environment", 1, environment_callback);
    ros::Publisher stop_line_pub_stack = n.advertise<common_msgs::StopLineList>("/path_planning/world_stop_lines", 1);
    stop_line_pub = &stop_line_pub_stack;

    ros::ServiceServer lanelet_route_service = n.advertiseService("/path_planning/" + ros::this_node::getName() + "/get_behavior", get_behavior);

    // global_reroute_publisher = &n.advertise<path_planning_msgs::GlobalCommand>(
    //     "/path_planning/reroute_request", 1);

    dynamic_reconfigure::Client<decision::DynamicConfig> config_client ("/decision_server");
    config_client.setConfigurationCallback(config_callback);
    Dynamic_Config.groups.behavioral_planning.LANE_CHANGE_LAT_DEV_CONDITION = 2;
    Dynamic_Config.groups.behavioral_planning.LANE_STABILIZED_HISTORY = 4;
    Dynamic_Config.groups.behavioral_planning.STOP_LINE_FORWARD_SEARCH = 35;    
    Dynamic_Config.groups.behavioral_planning.STOP_SIGN_DISTANCE_EPSILON = 3; 
    Dynamic_Config.groups.behavioral_planning.STOP_SIGN_TIME = 5;
    Dynamic_Config.groups.behavioral_planning.OBJECT_ROI_RADIUS = 5;
    Dynamic_Config.groups.behavioral_planning.OBJECT_STOPPING_DIST = 5;
    Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON = 0.2;
    Dynamic_Config.groups.behavioral_planning.OBJECT_STOPPED_TIMER = 5;

    ego_machine = std::unique_ptr<EgoStateMachine>(new EgoStateMachine(light_lock_publisher));
    global_command_machine = std::unique_ptr<GlobalCommandStateMachine>(new GlobalCommandStateMachine());
    world_manager = std::unique_ptr<WorldStateManager>(new WorldStateManager());
    
    // Generate triggers at 20HZ
    ros::Timer timer = n.createTimer(ros::Duration(1.f / 20), timer_callback);

    ros::spin();
     
    return 0;
}