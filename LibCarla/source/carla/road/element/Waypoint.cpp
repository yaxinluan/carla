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

  // ===========================================================================
  // -- Static local methods ---------------------------------------------------
  // ===========================================================================

  static const RoadSegment &GetRoad(const Map &map, id_type road_id) {
    auto *road = map.GetData().GetRoad(road_id);
    DEBUG_ASSERT(road != nullptr);
    return *road;
  }

  template <typename T>
  static std::vector<T> ConcatVectors(std::vector<T> dst, std::vector<T> src) {
    if (src.size() > dst.size()) {
      return ConcatVectors(src, dst);
    }
    dst.insert(
        dst.end(),
        std::make_move_iterator(src.begin()),
        std::make_move_iterator(src.end()));
    return dst;
  }

  // ===========================================================================
  // -- Waypoint ---------------------------------------------------------------
  // ===========================================================================

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
  }

  Waypoint::~Waypoint() = default;

  geom::Transform Waypoint::GetTransform() const {
    road::element::DirectedPoint dp =
        _map->GetData().GetRoad(_road_id)->GetDirectedPointIn(_dist);
    geom::Rotation rot(0.0, geom::Math::to_degrees(dp.tangent), 0.0);
    if (_lane_id > 0) {
      rot.yaw += 180.0;
    }

    auto *info = GetRoad(*_map, _road_id).GetInfo<RoadInfoLane>(0.0);
    DEBUG_ASSERT(info != nullptr);

    dp.ApplyLateralOffset(info->getLane(_lane_id)->_lane_center_offset);
    return geom::Transform(dp.location, rot);
  }

  RoadInfoList Waypoint::GetRoadInfo() const {
    return RoadInfoList(_map->GetData().GetRoad(_road_id)->GetInfos(_dist));
  }

  std::vector<Waypoint> Waypoint::Next(const double distance) const {
    DEBUG_ASSERT(_lane_id != 0);

    const auto &road_segment = GetRoad(*_map, _road_id);
    double distance_on_next_segment;

    if (_lane_id < 0) {
      // road goes forward.
      const auto total_distance = _dist + distance;
      const auto road_length = road_segment.GetLength();
      if (total_distance <= road_length) {
        return { Waypoint(_map, _road_id, _lane_id, total_distance) };
      }
      distance_on_next_segment = total_distance - road_length;
    } else {
      // road goes backward.
      const auto total_distance = _dist - distance;
      if (total_distance >= 0.0) {
        return { Waypoint(_map, _road_id, _lane_id, total_distance) };
      }
      distance_on_next_segment = std::abs(total_distance);
    }

    std::vector<Waypoint> result;
    const auto &next_lanes = _lane_id < 0 ? road_segment.GetNextLane(_lane_id) : road_segment.GetPrevLane(_lane_id);
    if (next_lanes.empty()) {
      log_error("no lanes!! lane id =", _lane_id, " road id=", _road_id);
    }
    for (auto &&pair : next_lanes) {
      auto lane_id = pair.first;
      auto road_id = pair.second;
      DEBUG_ASSERT(lane_id != 0);
      auto d = lane_id < 0 ? 0.0 : GetRoad(*_map, road_id).GetLength();
      auto waypoint = Waypoint(_map, road_id, lane_id, d);
      result = ConcatVectors(result, waypoint.Next(distance_on_next_segment));
    }
    return result;
  }

} // namespace element
} // namespace road
} // namespace carla
