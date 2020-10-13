#include "ego_state.hpp"

using namespace path_planning::behavioral;

path_planning_msgs::EgoVehicleState EgoState::getStateMsg() const {
    path_planning_msgs::EgoVehicleState msg;
    switch(_action) {
        case EgoStateAction::FOLLOWING_LANE:
            msg.ego_state = path_planning_msgs::EgoVehicleState::FOLLOWING_LANE;
            break;
        case EgoStateAction::CHANGING_LANE_LEFT:
            msg.ego_state = path_planning_msgs::EgoVehicleState::CHANGING_LANE;
            break;
        case EgoStateAction::CHANGING_LANE_RIGHT:
            msg.ego_state = path_planning_msgs::EgoVehicleState::CHANGING_LANE;
            break;
        case EgoStateAction::STOPPING:
            msg.ego_state = path_planning_msgs::EgoVehicleState::STOPPING;
            break;
        case EgoStateAction::WAITING_FOR_COMMANDS:
            msg.ego_state = path_planning_msgs::EgoVehicleState::WAITING_FOR_COMMANDS;
            break;
        default:
            throw std::domain_error("Unknown _action");
    }
    return msg;
}

std::string EgoState::serializeAction() const {
    switch(_action) {
        case EgoStateAction::CHANGING_LANE_LEFT:
            return "CHANGING_LANE_LEFT";
        case EgoStateAction::CHANGING_LANE_RIGHT:
            return "CHANGING_LANE_RIGHT";
        case EgoStateAction::FOLLOWING_LANE:
            return "FOLLOWING_LANE";
        case EgoStateAction::STOPPING:
            return "STOPPING";
        case EgoStateAction::WAITING_FOR_COMMANDS:
            return "WAITING_FOR_COMMANDS";
        default:
            return "UNKNOWN, update EgoState::serializeAction()";
    }
}

std::string EgoState::serialize() const {
    return "PATH_RESULT: " + _path_result.serializeResult() 
        + "\nEGO_ACTION: " + serializeAction()
        + (_action == EgoStateAction::STOPPING ? 
            ("\nStoppingDistance: " + std::to_string(_world_stop_result.getDistance())) : "");
}

EgoState EgoState::addLatDev(double dev) {
    while (_lat_dev_hist.size() > HISTORY_SIZE) {
        _lat_dev_hist.pop_back();
    }
    _lat_dev_hist.push_front(dev);
    return *this;
}

double EgoState::getLatestLatDev() const {
    if (_lat_dev_hist.size() > 0) {
        return _lat_dev_hist.front();
    }
    return 0;
}

EgoState EgoState::addDesiredLatDev(double dev) {
    while (_desired_lat_dev_hist.size() > HISTORY_SIZE) {
        _desired_lat_dev_hist.pop_back();
    }
    _desired_lat_dev_hist.push_front(dev);
    return *this;
}

double EgoState::getLatestDesiredLatDev() const {
    if (_desired_lat_dev_hist.size() > 0) {
        return _desired_lat_dev_hist.front();
    }
    return 0;
}

std::deque<double> EgoState::getLatDevHist() const {
    return _lat_dev_hist;
}

std::deque<double> EgoState::getDesiredLatDevHist() const {
    return _desired_lat_dev_hist;
}

EgoState::EgoState(EgoStateAction action): AbstractState(action), _cmd(GlobalCommand()){};

path_planning::VehicleState EgoState::getPose() const {
    return _pose;
}

EgoState EgoState::setPose(path_planning::VehicleState pose) {
    _pose = pose;
    return *this;
}

EgoState EgoState::setWorldStopResult(const StopResult& result) {
    _world_stop_result = result;
    return *this;
}

const StopResult& EgoState::getWorldStopResult() const {
    return _world_stop_result;
}

EgoState::PathResult EgoState::getPathResult() const {
    return _path_result;
}

bool EgoState::PathResult::hasPath() const {
    if (_result == PathResult::HAS_PATH) {
        return true;
    }
    return false;
}

optional<int> EgoState::PathResult::getLastLaneletID() const {
    if (_result != PathResult::HAS_PATH) {
        return boost::none;
    }
    return _path.back().id;
}

optional<std::deque<Lane>> EgoState::PathResult::getPath() const {
    if (_result != PathResult::HAS_PATH) {
        return boost::none;
    }
    return _path;
}

optional<path_planning::geom::Polyline> EgoState::PathResult::getFrenetReferenceLine() const {
    if (_result != PathResult::HAS_PATH) {
        return boost::none;
    }
    path_planning::geom::Polyline ret;
    bool laneChangeFirstLanelet = 
        _path.size() > 1
        && (_path[0].left_id == _path[1].id || _path[0].right_id == _path[1].id);

    for (unsigned int i = laneChangeFirstLanelet ? 1 : 0; i < _path.size(); i++) {
        ret.insert(ret.end(), _path[i].begin(), _path[i].end());
    }
    return ret;
}

void EgoState::PathResult::setPath(std::deque<Lane> path) {
    if (path.empty()) {
        throw std::invalid_argument("EgoState::PathResult::setPath called with empty path");
    }
    _path = path;
    _result = PathResult::HAS_PATH;
}

void EgoState::PathResult::setNoPath(RESULT_TYPE result, std::string what) {
    if (result == PathResult::HAS_PATH) {
        throw std::invalid_argument("EgoState::PathResult::setNoPath called with HAS_PATH result");
    }
    _path.clear();
    _result = result;
    _what_failure = what;
}

void EgoState::PathResult::prunePath(path_planning::VehicleState pose) {
    if (shouldPrunePath(pose)) {
        ROS_INFO("DONE SEGMENT");
        _path.pop_front();
        if (_path.empty()) {
            _result = PathResult::NONE;
        }
    }
}

EgoState EgoState::setGlobalCommand(const GlobalCommand &cmd){
    _cmd = cmd; 
    return *this; 
}

path_planning::GlobalCommand EgoState::getGlobalCommand(){
    return _cmd;
} 

bool EgoState::PathResult::shouldPrunePath(path_planning::VehicleState pose) const {
    // Nothing to prune
    if (!hasPath() || (_path.size() == 1 && _path[0].size() == 0)) {
        return false;
    }

    // If we are pruning only lanelet on path, check that we have completed traversing it
    if (_path.size() == 1) {
        return _path[0].back().distanceTo(pose) < Dynamic_Config.lane_cutoff_distance;
    }
    else {
        return 
            _path[0].distanceTo(pose) >= Dynamic_Config.lane_cutoff_distance
            && _path[1].distanceTo(pose) < Dynamic_Config.lane_cutoff_distance;
    }
}

std::string EgoState::PathResult::serializeResult() const {
    switch (_result) {
        case NONE:
            return "NONE";
        case UNKNOWN_FAILURE:
            return "UNKNOWN_FAILURE";
        case SEARCH_FAILURE:
            return "SEARCH_FAILURE";
        case NO_GLOBAL_COMMAND:
            return "NO_GLOBAL_COMMAND";
        case HAS_PATH:
            return "HAS_PATH";
    }
    return "UNKNOWN";
}

optional<std::deque<Lane>> EgoState::tryFindLaneletPath(
        Lane startLanelet, 
        path_planning::GlobalCommand goalCommand) {

    auto& destinations = goalCommand.link_id;
    
    // Already on desired link
    if (destinations.find(startLanelet.start_link_id) != destinations.end()){
        ROS_DEBUG_STREAM("Current link is a destination " << startLanelet);
        return std::deque<path_planning::Lane>({startLanelet});
    }

    std::queue<int> q;
    std::unordered_map<int, int> pred;
    int destID = 0;

    // Standard BFS, searching for any link id
    q.push(startLanelet.id);
    pred[startLanelet.id] = 0;

    // keep track of depth
    int currentDepth = 0, 
        elementsToDepthIncrease = 1, 
        nextElementsToDepthIncrease = 0;

    while (!q.empty()){
        int curID = q.front();
        path_planning::Lane curLane = queryLane(curID);
        q.pop();

        ROS_DEBUG_STREAM("Current " << curLane);

        // Always traverse "NEXT" before lane change
        std::vector<int> adj{curLane.next.begin(), curLane.next.end()};
        if (curLane.left_id != 0)
            adj.push_back(curLane.left_id);
        if (curLane.right_id != 0)
            adj.push_back(curLane.right_id);

        // Check depth
        nextElementsToDepthIncrease += adj.size();
        if (--elementsToDepthIncrease == 0) {
            if (Dynamic_Config.link_lookahead != -1 && ++currentDepth > Dynamic_Config.link_lookahead) {
                return boost::none;
            };
            elementsToDepthIncrease = nextElementsToDepthIncrease;
            nextElementsToDepthIncrease = 0;
        }

        for (auto& nextID : adj){
            ROS_DEBUG_STREAM("\t Next: " << nextID);
            // not yet visited
            if (pred.count(nextID) == 0){

                ROS_DEBUG_STREAM("\t\t UNVISITED:" << nextID);

                pred[nextID] = curID;
                q.push(nextID);
                if (destinations.find(queryLane(nextID).start_link_id) != destinations.end()){

                    ROS_DEBUG_STREAM("\t\t DESTINATION" << nextID);
                    destID = nextID;
                    break;
                }
            }
        }
        // Goal has been found
        if (destID != 0){
            break;
        }
    }

    // Goal has not been found
    if (destID == 0){
        return boost::none;
    }

    // extract path from pred
    std::stack<int> crawl;
    std::deque<path_planning::Lane> ret;
    while (pred.count(destID) != 0) {
        crawl.push(destID);
        destID = pred[destID];
    }
    while (!crawl.empty()) {
        ret.push_back(queryLane(crawl.top()));
        crawl.pop();
    }

    // If goal is a stop command, append path to rightmost lane
    if (goalCommand.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STOP){
        while (ret.back().right_id != 0){
            ret.push_back(queryLane(ret.back().right_id));
        }
    }
    return ret;
}

void EgoState::PathResult::printPath() const {
    ROS_DEBUG("path:");
    for (auto& lane : _path){
        ROS_DEBUG_STREAM("\t" << lane);
    }
}

path_planning::Lane EgoState::queryLane(int lane_id) {
    if (_topology.count(lane_id) == 0) {
        _topology = _hdm_interface.updateLaneTopology(_topology, lane_id, _pose);
    }
    return _topology.at(lane_id);
}

void EgoState::updatePath() {
    // If we have no goal, we can't plan a path
    if (_cmd.cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::NONE){
        _path_result.setNoPath(PathResult::NO_GLOBAL_COMMAND);
        return;
    }
    
    // We have a goal! Let's check to see if we need to plan a path to it now

    // First we prune our current path based on how much of it we have traversed
    _path_result.prunePath(_pose);

    // When there is no path or we have veered quite off from our current path, try to plan a new path from where we are now
    if (!_path_result.hasPath() || (distanceToPath() > Dynamic_Config.path_cutoff_distance && !_cmd.nearIntersection(_pose, Dynamic_Config.safe_intersection_distance))){
        ROS_INFO_THROTTLE(20, "BP: STARTUP OR RETRY");
        Lane current = _hdm_interface.getCurrentLane(_pose);
        optional<std::deque<Lane>> pathOrNone = tryFindLaneletPath(current, _cmd);
        if (pathOrNone) {
            _path_result.setPath(pathOrNone.get());
        }
        else {
            _path_result.setNoPath(PathResult::SEARCH_FAILURE);
        }
    }
}

// returns the minimum distance to the path
double EgoState::distanceToPath() const {
    if (!_path_result.hasPath()) {
        return 0;
    }
    double min = INT_MAX;
    std::deque<Lane> path = _path_result.getPath().get();
    for (auto& lane : path){
        int dist = lane.distanceTo(_pose);
        if (min > dist){
            min = dist;
        }
    }
    return min;
}