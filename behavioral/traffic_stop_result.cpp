#include "stop_result.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

StopResult StopResult::tryCreateWithStopLine(geom::Polyline ref_line, VehicleState ego_pose, geom::LineSegment stop_line) {
    if (ref_line.empty()) {
        // return default with no stop
        return {};
    }
    unsigned int egoIndex = ref_line.closestElementIndex(ego_pose);
    double dist = 0;
    for (unsigned int i = egoIndex; i < ref_line.size() - 1 && dist < Dynamic_Config.groups.behavioral_planning.STOP_LINE_FORWARD_SEARCH; i++) {
        path_planning::geom::LineSegment seg {ref_line[i], ref_line[i+1]};
        if (seg.intersects(stop_line)) {
            return {seg.p1, i, dist, ref_line};
        }
        dist += seg.length();
    }
    return {};
}

StopResult StopResult::tryCreateWithDynamicObject(
    geom::Polyline ref_line, VehicleState ego_pose, const Object& obj) {

    if (ref_line.empty()) {
        // return default with no stop
        return {};
    }
    
    unsigned int egoIndex = ref_line.closestElementIndex(ego_pose);
    unsigned int objIndex = ref_line.closestElementIndex(obj.center);

    double egoDistToObj = ref_line.distanceBetween(egoIndex, objIndex);
    double refDistToObj = obj.distanceTo(ref_line[objIndex]);
    if (egoDistToObj < 0 
        || egoDistToObj > Dynamic_Config.groups.behavioral_planning.STOP_LINE_FORWARD_SEARCH
        || refDistToObj > Dynamic_Config.groups.behavioral_planning.OBJECT_ROI_RADIUS) {
        return {};
    }

    double walkBackDist = 0;
    while (objIndex >= 1 && walkBackDist < Dynamic_Config.groups.behavioral_planning.OBJECT_STOPPING_DIST) {
        walkBackDist += geom::LineSegment(ref_line[objIndex], ref_line[objIndex - 1]).length();
        objIndex--;
    }

    return {ref_line[objIndex], objIndex, ref_line.distanceBetween(egoIndex, objIndex), ref_line};
}

StopResult StopResult::tryCreateWithStoppingLane(
    const geom::Polyline &ref_line, const Lane &lane, const VehicleState &ego_pose) {

    if (ref_line.empty()) {
        // return default with no stop
        return {};
    }

    // TODO: check if lane is stopping lane, for now assume all lanes are stopping lanes

    assert(ref_line.size() >= lane.size());
    unsigned int stopping_index = lane.size() - 1;
    unsigned int ego_index = ref_line.closestElementIndex(ego_pose);
    return {ref_line[stopping_index], stopping_index, ref_line.distanceBetween(ego_index, stopping_index), ref_line};
    

}

bool StopResult::hasStop() const {
    return _has_stop;
}

geom::Point2d StopResult::getStopPoint() const {
    assert(_has_stop);

    return _stop_point;
}

double StopResult::getDistance() const {
    assert(_has_stop);

    return _stopping_dist;
}

unsigned int StopResult::getPointIdx() const {
    assert(_has_stop);

    return _point_idx;
}

void StopResult::updateDistance(const VehicleState& new_ego_pose) {
    assert(_has_stop);

    _stopping_dist = _trajectory_reference.distanceBetween(
        _trajectory_reference.closestElementIndex(new_ego_pose),
        _point_idx);
}

StopResult::StopResult(geom::Point2d stop_point, unsigned int point_idx, 
    double stopping_dist, geom::Polyline trajectory_reference) : 
        _has_stop{true}, _stop_point{stop_point}, _point_idx{point_idx}, 
        _stopping_dist{stopping_dist}, _trajectory_reference{trajectory_reference} {}

StopResult::StopResult() : _has_stop{false} {}