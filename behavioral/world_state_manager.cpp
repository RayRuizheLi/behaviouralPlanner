#include "world_state_manager.hpp"

using namespace path_planning::behavioral;

void WorldStateManager::updateEnvironment(const path_planning::Environment &env) {
    for (const std::shared_ptr<path_planning::TrafficLight> &light : env.traffic_lights) {
        if (_traffic_lights.find(light->id) != _traffic_lights.end()) {
            _traffic_lights[light->id]->setState(_traffic_lights[light->id]->getState().setLightStatus(*light));
        }
        else {
            _traffic_lights.insert(std::make_pair(light->id, std::make_shared<TrafficLightStateMachine>(*light)));
        }
    }

    for (const std::shared_ptr<path_planning::TrafficSign> &sign : env.traffic_signs) {
        if(const std::shared_ptr<path_planning::StopSign> & stop_sign = std::dynamic_pointer_cast<StopSign>(sign)) {
            if (_stop_signs.find(stop_sign->id) != _stop_signs.end()) {
                _stop_signs[stop_sign->id]->setState(_stop_signs[stop_sign->id]->getState().setSign(*stop_sign));
            }
            else {
                _stop_signs.insert(std::make_pair(stop_sign->id, std::make_shared<StopSignStateMachine>(*stop_sign)));
            }
        }
    }

    for (const std::shared_ptr<OccupiableObject> &obj : env.obstacles) {
        if (const std::shared_ptr<path_planning::Pedestrian> & ped_obj = std::dynamic_pointer_cast<Pedestrian>(obj)) {
            if (_pedestrians.find(ped_obj->id) != _pedestrians.end()) {
                _pedestrians[ped_obj->id]->setState(_pedestrians[ped_obj->id]->getState().setPedestrianObj(*ped_obj));
            }
            else {
                _pedestrians.insert(std::make_pair(ped_obj->id, std::make_shared<PedestrianStateMachine>(*ped_obj)));
            }
        }
        else if (const std::shared_ptr<path_planning::Obstacle> & obs = std::dynamic_pointer_cast<Obstacle>(obj)) {
            if (_obstacles.find(obs->id) != _obstacles.end()) {
                _obstacles[obs->id]->setState(_obstacles[obs->id]->getState().setObstacle(*obs));
            }
            else {
                _obstacles.insert(std::make_pair(obs->id, std::make_shared<ObstacleStateMachine>(*obs)));
            }
        }
    }

    // TODO: Remove lights not in env from world?
}

void WorldStateManager::updateGlobalCommandQueue(path_planning::GlobalCommandQueue cmds) {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->setState(traffic_light.second->getState().setGlobalCommandQueue(cmds));
    }
}

void WorldStateManager::cycleMachines() {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->cycle();
    }

    for (auto const& stop_sign : _stop_signs) {
        stop_sign.second->cycle();
    }

    for (auto const& ped : _pedestrians) {
        ped.second->cycle();
    }

    for (auto const& obj : _obstacles) {
        obj.second->cycle();
    }
}

std::vector<path_planning::StopLine> WorldStateManager::getStopLines() const {
    std::vector<path_planning::StopLine> ret;
    for (auto const& traffic_light : _traffic_lights) {
        if (traffic_light.second->getState().getAction() == TrafficLightStateAction::NON_BLOCKING) {
            continue;
        }
        boost::optional<path_planning::StopLine> lineOrNone = traffic_light.second->getState().getStoppingLine();
        if (lineOrNone) {
            ret.push_back(lineOrNone.get());
        }
    }
    for (auto const& stop_sign : _stop_signs) {
        boost::optional<path_planning::StopLine> lineOrNone = stop_sign.second->getState().getStopLine();
        if (lineOrNone) {
            ret.push_back(lineOrNone.get());
        }
    }
    return ret;
}

StopResult WorldStateManager::getStopResult() const {
    unsigned int min_idx = UINT_MAX;
    StopResult ret;
    for (auto const& traffic_light : _traffic_lights) {
        if (traffic_light.second->getState().getAction() == TrafficLightStateAction::BLOCKING) {
            if (traffic_light.second->getState().getStopResult().getPointIdx() < min_idx) {
                ret = traffic_light.second->getState().getStopResult();
                min_idx = traffic_light.second->getState().getStopResult().getPointIdx();
            }
        }
    }
    for (auto const& stop_sign : _stop_signs) {
        if (stop_sign.second->getState().getAction() == StopSignStateAction::BLOCKING 
            || stop_sign.second->getState().getAction() == StopSignStateAction::ACTIVE_BLOCKING) {
            if (stop_sign.second->getState().getStopResult().getPointIdx() < min_idx) {
                ret = stop_sign.second->getState().getStopResult();
                min_idx = stop_sign.second->getState().getStopResult().getPointIdx();
            }
        }
    }
    for (auto const& ped : _pedestrians) {
        if (ped.second->getState().getAction() == PedestrianStateAction::ACTIVE 
            || ped.second->getState().getAction() == PedestrianStateAction::PAUSED) {
            if (ped.second->getState().getStopResult().getPointIdx() < min_idx) {
                ret = ped.second->getState().getStopResult();
                min_idx = ped.second->getState().getStopResult().getPointIdx();
            }
        }
    }
    for (auto const& obj : _obstacles) {
        if (obj.second->getState().getAction() == ObstacleStateAction::MOVING) {
            if (obj.second->getState().getStopResult().getPointIdx() < min_idx) {
                ret = obj.second->getState().getStopResult();
                min_idx = obj.second->getState().getStopResult().getPointIdx();
            }
        }
    }
    return ret;
}

void WorldStateManager::updateTrajectoryReference(const geom::Polyline& line) {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->setState(traffic_light.second->getState().setTrajectoryReference(line));
    }
    for (auto const& stop_sign : _stop_signs) {
        stop_sign.second->setState(stop_sign.second->getState().setTrajectoryReference(line));
    }
    for (auto const& ped : _pedestrians) {
        ped.second->setState(ped.second->getState().setTrajectoryReference(line));
    }
    for (auto const& obj : _obstacles) {
        obj.second->setState(obj.second->getState().setTrajectoryReference(line));
    }
}

void WorldStateManager::updateCurrentLane(const Lane& lane) {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->setState(traffic_light.second->getState().setCurrentLane(lane));
    }
}

void WorldStateManager::updateVehiclePose(const VehicleState& pose) {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->setState(traffic_light.second->getState().setVehiclePose(pose));
    }
    for (auto const& stop_sign : _stop_signs) {
        stop_sign.second->setState(stop_sign.second->getState().setVehiclePose(pose));
    }
    for (auto const& ped : _pedestrians) {
        ped.second->setState(ped.second->getState().setVehiclePose(pose));
    }
    for (auto const& obj : _obstacles) {
        obj.second->setState(obj.second->getState().setVehiclePose(pose));
    }
}

