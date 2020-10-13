#pragma once

#include "common/point2d.hpp"
#include "common/polyline.hpp"
#include "common/vehicle_state.hpp"
#include "common/line_segment.hpp"
#include "common/object.hpp"
#include "common/lane.hpp"
#include <DynamicNode/parameter_config.h>

namespace path_planning { namespace behavioral {

    class StopResult{
        public:
            StopResult();

            static StopResult tryCreateWithStopLine(
                geom::Polyline ref_line, VehicleState ego_pose, geom::LineSegment stop_line);

            static StopResult tryCreateWithDynamicObject(
                geom::Polyline ref_line, VehicleState ego_pose, const Object& obj);

            static StopResult tryCreateWithStoppingLane(
                const geom::Polyline &ref_line, const Lane &lane, const VehicleState &ego_pose);

            bool hasStop() const;
            geom::Point2d getStopPoint() const;
            unsigned int getPointIdx() const;
            double getDistance() const;
            void updateDistance(const VehicleState& new_ego_pose);

        private:
            StopResult(geom::Point2d stop_point, unsigned int point_idx, 
                double stopping_dist, geom::Polyline trajectory_reference);
            bool _has_stop = false;
            geom::Point2d _stop_point;
            unsigned int _point_idx;
            double _stopping_dist;
            geom::Polyline _trajectory_reference;

    }; 
}}
