// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/element/Waypoint.h"

#include "carla/Logging.h"
#include "carla/road/Map.h"

namespace carla {
namespace road {
namespace element {

  Waypoint::Waypoint(SharedPtr<const Map> m, const geom::Location &loc)
    : _map(m) {
    DEBUG_ASSERT(_map != nullptr);
    double nearest_dist = std::numeric_limits<double>::max();
    for (auto &&r : _map->GetData().GetRoadSegments()) {
      auto current_dist = r.GetNearestPoint(loc);
      if (current_dist.second < nearest_dist) {
        nearest_dist = current_dist.second;
        _road_id = r.GetId();
        _dist = current_dist.first;
      }
    }
    DEBUG_ASSERT(_dist <= _map->GetData().GetRoad(_road_id)->GetLength());
    const auto nearest_lane = _map->GetData().GetRoad(_road_id)->GetNearestLane(_dist, loc);
    _lane_id = nearest_lane;
    DEBUG_ASSERT(_lane_id != 0);
  }

  Waypoint::Waypoint(
      SharedPtr<const Map> map,
      id_type road_id,
      id_type lane_id,
      double distance)
    : _map(std::move(map)),
      _road_id(road_id),
      _lane_id(lane_id),
      _dist(distance) {
    DEBUG_ASSERT(_map != nullptr);
    DEBUG_ASSERT(_lane_id != 0);
  }

  Waypoint::~Waypoint() = default;

  geom::Transform Waypoint::GetTransform() const {
    road::element::DirectedPoint dp =
        _map->GetData().GetRoad(_road_id)->GetDirectedPointIn(_dist);
    geom::Rotation rot(0.0, geom::Math::to_degrees(dp.tangent), 0.0);
    if (_lane_id > 0) {
      rot.yaw += 180.0;
    }

    const auto *road_segment = _map->GetData().GetRoad(_road_id);
    DEBUG_ASSERT(road_segment != nullptr);
    const auto *info = road_segment->GetInfo<RoadInfoLane>(0.0);
    DEBUG_ASSERT(info != nullptr);

    dp.ApplyLateralOffset(info->getLane(_lane_id)->_lane_center_offset);
    return geom::Transform(dp.location, rot);
  }

  RoadInfoList Waypoint::GetRoadInfo() const {
    return RoadInfoList(_map->GetData().GetRoad(_road_id)->GetInfos(_dist));
  }

} // namespace element
} // namespace road
} // namespace carla
