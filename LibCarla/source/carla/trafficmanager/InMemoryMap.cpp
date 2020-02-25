// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/trafficmanager/InMemoryMap.h"

namespace carla {
namespace traffic_manager {

#define GRID_SIZE_FACTOR          2   // Related to GRID_SIZE 2^2
#define PED_GRID_SIZE            10  // Related to PED_GRID_SIZE

#define MakeGridId(_x, _y, _flag) _flag                    \
  ? std::make_pair                            \
    ( (static_cast<int>(_x) >> GRID_SIZE_FACTOR)            \
    , (static_cast<int>(_y) >> GRID_SIZE_FACTOR))            \
  : std::make_pair                            \
    ( (static_cast<int>(_x) / PED_GRID_SIZE)                \
    , (static_cast<int>(_y) / PED_GRID_SIZE))

#define MakeGridKey(_gkey) std::string(std::to_string(_gkey.first) + "#" + std::to_string(_gkey.second))

namespace MapConstants {

static const float INFINITE_DISTANCE = std::numeric_limits<float>::max();
static const float MAX_GEODESIC_GRID_LENGTH = 20.0f;
static const float TOPOLOGY_DISTANCE = 0.1f;
} // namespace MapConstants

namespace cg = carla::geom;
using namespace MapConstants;

InMemoryMap::InMemoryMap(WorldMap world_map) : _world_map(world_map) {}
InMemoryMap::~InMemoryMap() {}

SegmentId InMemoryMap::GetSegmentId(const WaypointPtr &wp) const {
  return std::make_tuple(wp->GetRoadId(), wp->GetLaneId(), wp->GetSectionId());
}

SegmentId InMemoryMap::GetSegmentId(const SimpleWaypointPtr &swp) const {
  return GetSegmentId(swp->GetWaypoint());
}

double InMemoryMap::GetLaneLength(const SimpleWaypointPtr &swp) const {
  return segmentLength.at(GetSegmentId(swp));
}

std::vector<SimpleWaypointPtr> InMemoryMap::GetSuccessors(const SegmentId segment_id,
    const SegmentTopology &segment_topology, SegmentMap &segment_map) {
  std::vector<SimpleWaypointPtr> result;
  if (segment_topology.find(segment_id) == segment_topology.end()) {
    return result;
  }

  for (const auto &successor_segment_id : segment_topology.at(segment_id).second) {
    if (segment_map.find(successor_segment_id) == segment_map.end()) {
      auto successors = GetSuccessors(successor_segment_id, segment_topology, segment_map);
      result.insert(result.end(), successors.begin(), successors.end());
    } else {
      result.emplace_back(segment_map.at(successor_segment_id).front());
    }
  }
  return result;
}

std::vector<SimpleWaypointPtr> InMemoryMap::GetPredecessors(const SegmentId segment_id,
    const SegmentTopology &segment_topology, SegmentMap &segment_map) {
  std::vector<SimpleWaypointPtr> result;
  if (segment_topology.find(segment_id) == segment_topology.end()) {
    return result;
  }

  for (const auto &predecessor_segment_id : segment_topology.at(segment_id).first) {
    if (segment_map.find(predecessor_segment_id) == segment_map.end()) {
      auto predecessors = GetPredecessors(predecessor_segment_id, segment_topology, segment_map);
      result.insert(result.end(), predecessors.begin(), predecessors.end());
    } else {
      result.emplace_back(segment_map.at(predecessor_segment_id).back());
    }
  }
  return result;
}

void InMemoryMap::SetUp() {

  // 1. Building segment topology (i.e., defining set of segment predecessors and successors)
  auto waypoint_topology = _world_map->GetTopology();

  SegmentTopology segment_topology;
  std::unordered_map<int64_t, std::pair<std::set<crd::RoadId>, std::set<crd::RoadId>>> std_road_connectivity;
  std::unordered_map<crd::RoadId, bool> is_real_junction;

  for (auto &connection : waypoint_topology) {
    auto &waypoint = connection.first;
    auto &successor = connection.second;

    // Setting segment predecessors and successors.
    SegmentId waypoint_segment_id = GetSegmentId(connection.first);
    SegmentId successor_segment_id = GetSegmentId(connection.second);
    segment_topology[waypoint_segment_id].second.push_back(successor_segment_id);
    segment_topology[successor_segment_id].first.push_back(waypoint_segment_id);

    // From path to standard road.
    if (waypoint->IsJunction() && !successor->IsJunction()) {
      crd::RoadId path_id = waypoint->GetRoadId();
      int64_t std_road_id = static_cast<int64_t>(successor->GetRoadId());
      std_road_id = (successor->GetLaneId() < 0) ? -1 * std_road_id : std_road_id;

      std_road_connectivity[std_road_id].first.insert(path_id);
      auto &in_paths = std_road_connectivity[std_road_id].first;
      if (in_paths.size() >= 2) {
        std::for_each(in_paths.begin(), in_paths.end(), [&is_real_junction](crd::RoadId id) {is_real_junction[id] = true;});
      }
    }

    // From standard road to path.
    if (!waypoint->IsJunction() && successor->IsJunction()) {
      crd::RoadId path_id = successor->GetRoadId();
      int64_t std_road_id = static_cast<int64_t>(waypoint->GetRoadId());
      std_road_id = (waypoint->GetLaneId() < 0) ? -1 * std_road_id : std_road_id;

      std_road_connectivity[std_road_id].second.insert(path_id);
      auto &out_paths = std_road_connectivity[std_road_id].second;
      if (out_paths.size() >= 2) {
        std::for_each(out_paths.begin(), out_paths.end(), [&is_real_junction](crd::RoadId id) {is_real_junction[id] = true;});
      }
    }
  }

  /// Clear any previopus data
  SegmentMap segment_map;

  // 2. Consuming the raw dense topology from cc::Map into SimpleWaypoints.
  auto raw_dense_topology = _world_map->GenerateWaypoints(TOPOLOGY_DISTANCE);
  for (auto &waypoint_ptr: raw_dense_topology) {
    segment_map[GetSegmentId(waypoint_ptr)].emplace_back(std::make_shared<SimpleWaypoint>(waypoint_ptr));
  }

  auto square = [](float input) {return std::pow(input, 2);};
  auto compare_s = [](const SimpleWaypointPtr &swp1, const SimpleWaypointPtr &swp2) {
    return (swp1->GetWaypoint()->GetDistance() < swp2->GetWaypoint()->GetDistance());
  };

  GeoGridId geodesic_grid_id_counter = -1;
  for (auto &segment: segment_map) {
    auto &segment_waypoints = segment.second;

    // Generating geodesic grid ids.
    ++geodesic_grid_id_counter;

    // Ordering waypoints according to road direction.
    std::sort(segment_waypoints.begin(), segment_waypoints.end(), compare_s);
    auto lane_id = segment_waypoints.front()->GetWaypoint()->GetLaneId();
    if (lane_id > 0) {
      std::reverse(segment_waypoints.begin(), segment_waypoints.end());
    }

    // Placing intra-segment connections.
    cg::Location grid_edge_location = segment_waypoints.front()->GetLocation();
    for (std::size_t i = 0; i < segment_waypoints.size() - 1; ++i) {

      // Assigning grid id.
      if (cg::Math::DistanceSquared(grid_edge_location, segment_waypoints.at(i)->GetLocation()) >
      square(MAX_GEODESIC_GRID_LENGTH)) {
        ++geodesic_grid_id_counter;
        grid_edge_location = segment_waypoints.at(i)->GetLocation();
      }
      segment_waypoints.at(i)->SetGeodesicGridId(geodesic_grid_id_counter);

      segment_waypoints.at(i)->SetNextWaypoint({segment_waypoints.at(i + 1)});
      segment_waypoints.at(i + 1)->SetPreviousWaypoint({segment_waypoints.at(i)});
    }
    segment_waypoints.back()->SetGeodesicGridId(geodesic_grid_id_counter);

    // Adding simple waypoints to processed dense topology.
    for (auto swp: segment_waypoints) {
      // Checking whether the waypoint is a real junction.
      auto road_id = swp->GetWaypoint()->GetRoadId();
      if (swp->GetWaypoint()->IsJunction() && !is_real_junction.count(road_id)) {
        swp->SetIsJunction(false);
      } else {
        swp->SetIsJunction(swp->GetWaypoint()->IsJunction());
      }

      dense_topology.push_back(swp);
    }
  }

  // Localizing waypoints into grids.
  for (auto &simple_waypoint: dense_topology) {
    if (simple_waypoint != nullptr) {
      const cg::Location loc = simple_waypoint->GetLocation();

      /// Get grid point for vehicle
      const std::pair<int, int> gidV = MakeGridId(loc.x, loc.y, true);
      const std::string grid_key = MakeGridKey(gidV);
      if (waypoint_grid.find(grid_key) == waypoint_grid.end()) {
        waypoint_grid.insert({grid_key, {simple_waypoint}});
      } else {
        waypoint_grid.at(grid_key).insert(simple_waypoint);
      }

      /// Get grid point for pedestrian
      const std::pair<int, int> gidP = MakeGridId(loc.x, loc.y, false);
      const std::string ped_grid_key = MakeGridKey(gidP);
      if (ped_waypoint_grid.find(ped_grid_key) == ped_waypoint_grid.end()) {
        ped_waypoint_grid.insert({ped_grid_key, {simple_waypoint}});
      } else {
        ped_waypoint_grid.at(ped_grid_key).insert(simple_waypoint);
      }
    }
  }

  // Placing inter-segment connections.
  for (auto &segment : segment_map) {
    SegmentId segment_id = segment.first;
    auto &segment_waypoints = segment.second;

    auto successors = GetSuccessors(segment_id, segment_topology, segment_map);
    auto predecessors = GetPredecessors(segment_id, segment_topology, segment_map);

    segment_waypoints.front()->SetPreviousWaypoint(predecessors);
    segment_waypoints.back()->SetNextWaypoint(successors);
  }

  // Linking lane change connections.
  for (auto &simple_waypoint:dense_topology) {
    if (!simple_waypoint->CheckJunction()) {
      FindAndLinkLaneChange(simple_waypoint);
    }
  }

  // Linking any unconnected segments.
  for (auto &swp: dense_topology) {
    if (swp->GetNextWaypoint().empty()) {
      auto neighbour = swp->GetRightWaypoint();
      if (!neighbour) {
        neighbour = swp->GetLeftWaypoint();
      }

      if (neighbour) {
        swp->SetNextWaypoint(neighbour->GetNextWaypoint());
        for (auto next_waypoint : neighbour->GetNextWaypoint()) {
          next_waypoint->SetPreviousWaypoint({swp});
        }
      }
    }
  }

  MakeGeodesiGridCenters();


  /// Keep segment distance details
  for (auto &segment : segment_map) {
	  segmentLength[segment.first] = segment.second.size();
  }
}

SimpleWaypointPtr InMemoryMap::GetWaypointInVicinity(cg::Location location) {

  const std::pair<int, int> grid_ids = MakeGridId(location.x, location.y, true);
  SimpleWaypointPtr closest_waypoint = nullptr;
  float closest_distance = INFINITE_DISTANCE;

  // Search 3x3 surrounding grids for closest waypoint.
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {

      /// for current grid point
      const std::pair<int, int> gidV({grid_ids.first + i, grid_ids.second + j});
      const std::string grid_key = MakeGridKey(gidV);

      /// If valid grid point present
      if (waypoint_grid.find(grid_key) != waypoint_grid.end()) {

        /// Get way point set
        const auto &waypoint_set = waypoint_grid.at(grid_key);

        /// For all points in the set find the closest one
        for (auto &simple_waypoint: waypoint_set) {

          // Only if it is in the same horizontal plane as the requested location.
          if((simple_waypoint->GetLocation().z - location.z) < 1.0) {

            /// Get distance
            auto distance = simple_waypoint->DistanceSquared(location);

            /// Store smallest distance way-point
            if (distance < closest_distance) {
              closest_waypoint = simple_waypoint;
              closest_distance = distance;
            }
          }
        }
      }
    }
  }

  return closest_waypoint;

}

SimpleWaypointPtr InMemoryMap::GetPedWaypoint(cg::Location location) {

  const std::pair<int, int> grid_ids = MakeGridId(location.x, location.y, false);
  SimpleWaypointPtr closest_waypoint = nullptr;
  float closest_distance = INFINITE_DISTANCE;

  // Search 3x3 surrounding grids for closest waypoint.
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {

      /// for current grid point
      const std::pair<int, int> gidP({grid_ids.first + i, grid_ids.second + j});
      const std::string grid_key = MakeGridKey(gidP);

      /// If valid grid point present
      if (ped_waypoint_grid.find(grid_key) != ped_waypoint_grid.end()) {

        /// Get way point set
        const auto &waypoint_set = ped_waypoint_grid.at(grid_key);

        /// For all points in the set find the closest one
        for (auto &simple_waypoint: waypoint_set) {

          /// Get distance
          auto distance = simple_waypoint->DistanceSquared(location);

          /// Store smallest distance
          if (distance < closest_distance) {
            closest_waypoint = simple_waypoint;
            closest_distance = distance;
          }
        }
      }
    }
  }

  return closest_waypoint;
}

SimpleWaypointPtr InMemoryMap::GetWaypoint(const cg::Location &location) const {

  SimpleWaypointPtr closest_waypoint;
  float min_distance = INFINITE_DISTANCE;
  for (auto &simple_waypoint : dense_topology) {
    const float current_distance = simple_waypoint->DistanceSquared(location);
    if (current_distance < min_distance) {
      min_distance = current_distance;
      closest_waypoint = simple_waypoint;
    }
  }
  return closest_waypoint;
}

std::vector<SimpleWaypointPtr> InMemoryMap::GetDenseTopology() const {
  return dense_topology;
}

void InMemoryMap::FindAndLinkLaneChange(SimpleWaypointPtr reference_waypoint) {

  const WaypointPtr raw_waypoint = reference_waypoint->GetWaypoint();
  const crd::element::LaneMarking::LaneChange lane_change = raw_waypoint->GetLaneChange();
  const auto change_right = crd::element::LaneMarking::LaneChange::Right;
  const auto change_left = crd::element::LaneMarking::LaneChange::Left;
  const auto change_both = crd::element::LaneMarking::LaneChange::Both;

  try {
    if (lane_change == change_right || lane_change == change_both) {

      const WaypointPtr right_waypoint =  raw_waypoint->GetRight();
      if (right_waypoint != nullptr &&
          right_waypoint->GetType() == crd::Lane::LaneType::Driving &&
          (right_waypoint->GetLaneId() * raw_waypoint->GetLaneId() > 0)) {

        SimpleWaypointPtr closest_simple_waypoint =
            GetWaypointInVicinity(right_waypoint->GetTransform().location);
        if (closest_simple_waypoint == nullptr) {
          closest_simple_waypoint = GetWaypoint(right_waypoint->GetTransform().location);
        }
        reference_waypoint->SetRightWaypoint(closest_simple_waypoint);
      }
    }

    if (lane_change == change_left || lane_change == change_both) {

      const WaypointPtr left_waypoint =  raw_waypoint->GetLeft();
      if (left_waypoint != nullptr &&
          left_waypoint->GetType() == crd::Lane::LaneType::Driving &&
          (left_waypoint->GetLaneId() * raw_waypoint->GetLaneId() > 0)) {

        SimpleWaypointPtr closest_simple_waypoint =
            GetWaypointInVicinity(left_waypoint->GetTransform().location);
        if (closest_simple_waypoint == nullptr) {
          closest_simple_waypoint = GetWaypoint(left_waypoint->GetTransform().location);
        }
        reference_waypoint->SetLeftWaypoint(closest_simple_waypoint);
      }
    }
  } catch (const std::invalid_argument &e) {
    cg::Location loc = reference_waypoint->GetLocation();
    carla::log_info(
        "Unable to link lane change connection at: " +
        std::to_string(loc.x) + " " +
        std::to_string(loc.y) + " " +
        std::to_string(loc.z));
  }
}

void InMemoryMap::MakeGeodesiGridCenters() {
  for (auto &swp: dense_topology) {
    GeoGridId ggid = swp->CheckJunction() ? swp->GetJunctionId() : swp->GetGeodesicGridId();
    if (geodesic_grid_center.find(ggid) == geodesic_grid_center.end()) {
      geodesic_grid_center.insert({ggid, swp->GetLocation()});
    } else {
      cg::Location &grid_loc = geodesic_grid_center.at(ggid);
      grid_loc = (grid_loc + swp->GetLocation()) / 2;
    }
  }
}

cg::Location InMemoryMap::GetGeodesicGridCenter(GeoGridId ggid) {
  cg::Location grid_center;
  if (geodesic_grid_center.find(ggid) != geodesic_grid_center.end()) {
    grid_center = geodesic_grid_center.at(ggid);
  } else {
    grid_center = cg::Location();
  }
  return grid_center;
}

} // namespace traffic_manager
} // namespace carla
