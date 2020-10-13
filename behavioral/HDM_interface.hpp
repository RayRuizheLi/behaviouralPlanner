#pragma once

#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <exception>
#include <string>
#include <deque>

#include "DynamicNode/parameter_config.h"
#include "common/environment.hpp"
#include "common/lane.hpp"
#include "common/point2d.hpp"
#include "common/polyline.hpp"
#include "common/road_data.hpp"
#include "common/ros_msgs.hpp"

#include <common_msgs/HDMLane.h>
#include <common_msgs/HDMLaneList.h>
#include <hdmap_processing/GetConnectedLanes.h>
#include <hdmap_processing/GetCurrentLane.h>
#include <hdmap_processing/GetCurrentLaneId.h>
#include <hdmap_processing/GetLane.h>

namespace path_planning{

class ServiceFailed : public std::exception {
  public:
  std::string serviceName;
  ServiceFailed(std::string s) : serviceName(s) {}
  const char* what() const throw()
  {
    return ("Service Failed " + serviceName).c_str();
  }
};

class HDMInterface {
  // HDMap Service library
  hdmap_processing::GetConnectedLanes getConnectedLanesSrv;
  hdmap_processing::GetCurrentLane getCurrentLaneSrv;
  hdmap_processing::GetCurrentLaneId getCurrentLaneIdSrv;
  hdmap_processing::GetLane getLaneSrv;

  public:
  virtual Lane getLane(int lane_id);
  virtual Lane getCurrentLane(VehicleState ego_pose);
  virtual LaneTopology updateLaneTopology(LaneTopology topology, int lane_id, VehicleState ego_pose);
  LaneTopology trimLaneAdjacencies(LaneTopology topology);

  virtual ~HDMInterface() {};
};

}
