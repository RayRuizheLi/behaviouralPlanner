
#include "HDM_interface.hpp"
#include "common/road_data.hpp"

namespace path_planning {
  Lane HDMInterface::getLane(int lane_id){
    getLaneSrv.request.lane_id = lane_id;
    if (!ros::service::call("/hdm_liblanelet_service/get_lane", getCurrentLaneSrv)) {
      ROS_ERROR("get_lane service failed for id: %d", lane_id);
      throw ServiceFailed("get_lane");
    }
    return ros_msgs::parseHDMLane(getLaneSrv.response.hdm_lane);
  }
  Lane HDMInterface::getCurrentLane(VehicleState ego_pose){
    getCurrentLaneSrv.request.pose = ros_msgs::generatePoseMsg(ego_pose);
    if (!ros::service::call("/hdm_liblanelet_service/get_current_lane", getCurrentLaneSrv)) {
      ROS_ERROR("get_current_lane service failed");
      throw ServiceFailed("get_current_lane");
    }
    return ros_msgs::parseHDMLane(getCurrentLaneSrv.response.hdm_lane);
  }
  LaneTopology HDMInterface::updateLaneTopology(LaneTopology topology, int lane_id, VehicleState ego_pose){
    if (lane_id == 0){
      Lane cur = getCurrentLane(ego_pose);
      lane_id = cur.id;
    }

    getConnectedLanesSrv.request.lane_id = lane_id;
    if (!ros::service::call("/hdm_liblanelet_service/get_connected_lanes", getConnectedLanesSrv)) {
      ROS_ERROR("get_connected_lanes service failed for id: %d", lane_id);
      throw ServiceFailed("get_connected_lanes");
    }
    auto newTopology = ros_msgs::parseHDMLaneList(getConnectedLanesSrv.response.hdm_lanes);
    topology.insert(newTopology.begin(), newTopology.end());
    return trimLaneAdjacencies(topology);
  }
  LaneTopology HDMInterface::trimLaneAdjacencies(LaneTopology topology){
    for (auto& pair : topology){
      Lane& lane = pair.second;
      if (lane.borders.right.road_type != path_planning::ROAD_TYPE::CROSSABLE){
        lane.right_id = 0;
      }
      if (lane.borders.left.road_type != path_planning::ROAD_TYPE::CROSSABLE){
        lane.left_id = 0;
      }
    }
    return topology;
  }
}
