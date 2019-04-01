// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/element/LaneCrossingCalculator.h"

#include "carla/geom/Location.h"
#include "carla/road/Map.h"
#include "carla/ListView.h"
#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/WaypointInformationTypes.h"

namespace carla {
namespace road {
namespace element {

  /// Calculate the lane markings that need to be crossed from @a lane_id_origin
  /// to @a lane_id_destination.
  ///
  /// @todo This should use the info in the OpenDrive instead.
  static std::vector<LaneMarking> CrossingAtSameSection(
      const Map &map,
      const Waypoint *wp_origin,
      bool origin_is_at_left) {
    auto marks_wp_origin = map.GetMarkRecord(*wp_origin);

    if (origin_is_at_left) {
      if (wp_origin->lane_id > 0.0) {
        // inner lane mark
        const auto r = WaypointInfoRoadMark(*marks_wp_origin.second).type;
        if (r != LaneMarking::None) {
          return { r };
        }
      } else {
        // outer lane mark
        const auto r = WaypointInfoRoadMark(*marks_wp_origin.first).type;
        if (r != LaneMarking::None) {
          return { r };
        }
      }
    } else {
      if (wp_origin->lane_id > 0.0) {
        // outer lane mark
        const auto r = WaypointInfoRoadMark(*marks_wp_origin.first).type;
        if (r != LaneMarking::None) {
          return { r };
        }
      } else {
        // inner lane mark
        const auto r = WaypointInfoRoadMark(*marks_wp_origin.second).type;
        if (r != LaneMarking::None) {
          return { r };
        }
      }
    }
    return {};
  }

  // static bool IsOffRoad(const Map &map, const geom::Location &location) {
  //   return !map.GetWaypoint(location).has_value();
  // }

  std::vector<LaneMarking> LaneCrossingCalculator::Calculate(
      const Map &map,
      const geom::Location &origin,
      const geom::Location &destination) {
    const auto w0 = map.GetClosestWaypointOnRoad(origin, static_cast<uint32_t>(Lane::LaneType::Any));
    const auto w1 = map.GetClosestWaypointOnRoad(destination, static_cast<uint32_t>(Lane::LaneType::Any));

    if (!w0.has_value() || !w1.has_value()) {
      return {};
    }
    if (w0->road_id != w1->road_id || w0->section_id != w1->section_id) {
      /// @todo This case should also be handled.
      return {};
    }
    if (map.IsJunction(w0->road_id) || map.IsJunction(w1->road_id)) {
      return {};
    }
    if (w0->lane_id == w1->lane_id) {
      // the waypoints are in the same lane
      return {};
    }

    return CrossingAtSameSection(map, &*w0, (w0->lane_id > w1->lane_id));
  }

} // namespace element
} // namespace road
} // namespace carla
