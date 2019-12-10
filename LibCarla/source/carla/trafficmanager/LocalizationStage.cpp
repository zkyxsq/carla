#include "LocalizationStage.h"

namespace traffic_manager {

namespace LocalizationConstants {
  static const float WAYPOINT_TIME_HORIZON = 3.0f;
  static const float MINIMUM_HORIZON_LENGTH = 25.0f;
  static const float TARGET_WAYPOINT_TIME_HORIZON = 0.5f;
  static const float TARGET_WAYPOINT_HORIZON_LENGTH = 2.0f;
  static const float MINIMUM_JUNCTION_LOOK_AHEAD = 6.0f;
  static const float HIGHWAY_SPEED = 50.0f;
}
  using namespace LocalizationConstants;

  LocalizationStage::LocalizationStage(
      std::shared_ptr<LocalizationToPlannerMessenger> planner_messenger,
      std::shared_ptr<LocalizationToCollisionMessenger> collision_messenger,
      std::shared_ptr<LocalizationToTrafficLightMessenger> traffic_light_messenger,
      AtomicActorSet &registered_actors,
      InMemoryMap &local_map,
      Parameters &parameters,
      cc::DebugHelper &debug_helper)
    : planner_messenger(planner_messenger),
      collision_messenger(collision_messenger),
      traffic_light_messenger(traffic_light_messenger),
      registered_actors(registered_actors),
      local_map(local_map),
      parameters(parameters),
      debug_helper(debug_helper) {

    // Initializing various output frame selectors.
    planner_frame_selector = true;
    collision_frame_selector = true;
    collision_frame_ready = false;
    traffic_light_frame_selector = true;
    // Initializing the number of vehicles to zero in the begining.
    number_of_vehicles = 0u;

    // Initializing messenger states to initiate data writes
    // preemptively since this is the first stage in the pipeline.
    planner_messenger_state = planner_messenger->GetState() - 1;
    collision_messenger_state = collision_messenger->GetState() - 1;
    traffic_light_messenger_state = traffic_light_messenger->GetState() - 1;
    // Initializing the registered actors container state.
    registered_actors_state = -1;

    // Initialize all to false
    for (uint i = 0u; i < actor_list.size(); ++i) {
      
      Actor vehicle = actor_list.at(i);
      ActorId actor_id = vehicle->GetId();
      Approached[actor_id] = false;
      final_points[actor_id] = nullptr;
      }
  }

  LocalizationStage::~LocalizationStage() {}

  void LocalizationStage::Action() {

    // Selecting output frames based on selector keys.
    auto current_planner_frame = planner_frame_selector ? planner_frame_a : planner_frame_b;
    auto current_collision_frame = collision_frame_selector ? collision_frame_a : collision_frame_b;
    auto current_traffic_light_frame =
        traffic_light_frame_selector ? traffic_light_frame_a : traffic_light_frame_b;

    // Looping over registered actors.
    for (uint i = 0u; i < actor_list.size(); ++i) {

      Actor vehicle = actor_list.at(i);
      ActorId actor_id = vehicle->GetId();
      cg::Location vehicle_location = vehicle->GetLocation();
      float vehicle_velocity = vehicle->GetVelocity().Length();

      float horizon_size = std::max(
          WAYPOINT_TIME_HORIZON * vehicle_velocity,
          MINIMUM_HORIZON_LENGTH);

      Buffer &waypoint_buffer = buffer_list->at(i);

      // Purge passed waypoints.
      if (!waypoint_buffer.empty()) {

        float dot_product = DeviationDotProduct(vehicle, waypoint_buffer.front()->GetLocation());

        while (dot_product <= 0 && !waypoint_buffer.empty()) {
          waypoint_buffer.pop_front();
          if (!waypoint_buffer.empty()) {
            dot_product = DeviationDotProduct(vehicle, waypoint_buffer.front()->GetLocation());
          }
        }
      }

      // Initializing buffer if it is empty.
      if (waypoint_buffer.empty()) {
        SimpleWaypointPtr closest_waypoint = local_map.GetWaypointInVicinity(vehicle_location);
        if (closest_waypoint == nullptr) {
          closest_waypoint = local_map.GetWaypoint(vehicle_location);
        }
        waypoint_buffer.push_back(closest_waypoint);
      }

      // Assign a lane change.
      SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
      GeoIds current_road_ids = {
        front_waypoint->GetWaypoint()->GetRoadId(),
        front_waypoint->GetWaypoint()->GetSectionId(),
        front_waypoint->GetWaypoint()->GetLaneId()
      };

      traffic_distributor.UpdateVehicleRoadPosition(actor_id, current_road_ids);
      ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(vehicle);
      bool force_lane_change = lane_change_info.change_lane;
      bool lane_change_direction = lane_change_info.direction;

      if ((parameters.GetAutoLaneChange(vehicle) || force_lane_change) &&
          !front_waypoint->CheckJunction()) {

        SimpleWaypointPtr change_over_point = traffic_distributor.AssignLaneChange(
            vehicle, front_waypoint, current_road_ids,
            buffer_list, vehicle_id_to_index,
            actor_list, debug_helper, force_lane_change, lane_change_direction);

        if (change_over_point != nullptr) {
          waypoint_buffer.clear();
          waypoint_buffer.push_back(change_over_point);
        }
      }

      // Populating the buffer.
      while (waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front())
          <= std::pow(horizon_size, 2)) {

        std::vector<SimpleWaypointPtr> next_waypoints = waypoint_buffer.back()->GetNextWaypoint();
        uint selection_index = 0u;
        // Pseudo-randomized path selection if found more than one choice.
        if (next_waypoints.size() > 1) {
          selection_index = static_cast<uint>(rand()) % next_waypoints.size();
        }

        waypoint_buffer.push_back(next_waypoints.at(selection_index));
      }

      // Generating output.
      float target_point_distance = std::max(std::ceil(vehicle_velocity * TARGET_WAYPOINT_TIME_HORIZON),
          TARGET_WAYPOINT_HORIZON_LENGTH);
      SimpleWaypointPtr target_waypoint = waypoint_buffer.front();
      for (uint j = 0u;
          (j < waypoint_buffer.size()) &&
          (waypoint_buffer.front()->DistanceSquared(target_waypoint)
          < std::pow(target_point_distance, 2));
          ++j) {
        target_waypoint = waypoint_buffer.at(j);
      }
      cg::Location target_location = target_waypoint->GetLocation();
      float dot_product = DeviationDotProduct(vehicle, target_location);
      float cross_product = DeviationCrossProduct(vehicle, target_location);
      dot_product = 1 - dot_product;
      if (cross_product < 0) {
        dot_product *= -1;
      }

      /* Calculate the distance between the car and the trajectory (TODO: Use in the PID)
      auto Vehicle = boost::static_pointer_cast<cc::Vehicle>(vehicle);
      // Calculate the parameters of the line
      cg::Vector3D velocity = Vehicle->GetVelocity();
      velocity.z = 0;
      cg::Location trajectory_location_ini = waypoint_buffer.front()->GetLocation();
      trajectory_location_ini.z = 0;
      cg::Location trajectory_location_fin = target_location;
      trajectory_location_fin.z = 0;
      cg::Vector3D trajectory_vector = trajectory_location_fin - trajectory_location_ini;
      float a = trajectory_vector.x;
      float b = -trajectory_vector.y;
      float c = b*trajectory_location_ini.y - a*trajectory_location_ini.x;
      // Distance from the vehicle to the desired trajectory
      float distance = (a*vehicle_location.x + b*vehicle_location.y + c) / 
                        std::sqrt(std::pow(a,2.0f)+std::pow(b,2.0f));*/
      float distance = 0; // TODO: use in PID

      // Filtering out false junctions on highways.
      // On highways, if there is only one possible path and the section is
      // marked as intersection, ignore it.
      auto vehicle_reference = boost::static_pointer_cast<cc::Vehicle>(vehicle);
      float speed_limit = vehicle_reference->GetSpeedLimit();
      float look_ahead_distance = std::max(2 * vehicle_velocity, MINIMUM_JUNCTION_LOOK_AHEAD);

      SimpleWaypointPtr look_ahead_point = waypoint_buffer.front();
      uint look_ahead_index = 0u;
      for (uint j = 0u;
          (waypoint_buffer.front()->DistanceSquared(look_ahead_point)
          < std::pow(look_ahead_distance, 2)) &&
          (j < waypoint_buffer.size());
          ++j) {
        look_ahead_point = waypoint_buffer.at(j);
        look_ahead_index = j;
      }

      bool approaching_junction = false;
      if (waypoint_buffer.front()->CheckJunction() ||
          (look_ahead_point->CheckJunction() && !(waypoint_buffer.front()->CheckJunction()))) {
        if (speed_limit > HIGHWAY_SPEED) {
          for (uint j = 0u; (j < look_ahead_index) && !approaching_junction; ++j) {
            SimpleWaypointPtr swp = waypoint_buffer.at(j);
            if (swp->GetNextWaypoint().size() > 1) {
              approaching_junction = true;
            }
          }
        } else {
          approaching_junction = true;
        }
      }

      // Reset the variable
      if (!approaching_junction){
        // No need to check if the actor exists as if it doesn't, it will be added
        Approached[actor_id] = false;
        final_points[actor_id] = nullptr;
      }
      
      // If it's coming (and 2 more ifs), extend the buffer until passed the junction
      if (approaching_junction &&
         (Approached.find(actor_id) != Approached.end()) &&
          Approached[actor_id] == false){  

        Approached[actor_id] = true;

        // Length of the car
        float length = Vehicle->GetBoundingBox().extent.x;
        // First Waypoint after the junction
        SimpleWaypointPtr safe_point;
        // First Waypoint after the junction
        SimpleWaypointPtr final_point;
        // Safe space after the junction
        float safe_distance = 2*length + parameters.GetDistanceToLeadingVehicle(vehicle);
        // Multiple junction detection. TODO: check for false intersections
        bool extend_buffer = true;

        while(extend_buffer == true){
          extend_buffer = false;

          // Case 1) The buffer has to be extended (It stops at the intersection)
          if (waypoint_buffer.back()->CheckJunction()){

            // Keep adding waypoints until the junction is over.
            while (waypoint_buffer.back()->CheckJunction()) {

              // Record the last point as a safe one and safe it
              safe_point = waypoint_buffer.back()->GetNextWaypoint().front();
              waypoint_buffer.push_back(safe_point);
            }

            final_point = waypoint_buffer.back()->GetNextWaypoint().front();
            // Extend the buffer a little more. Keep adding waypoints
            // until the actor is able to fit after the junction.
            while(final_point->Distance(safe_point->GetLocation()) < safe_distance){
              if(waypoint_buffer.back()->CheckJunction()){
                extend_buffer = true;
                break;
              }
              waypoint_buffer.push_back(final_point);
              final_point = waypoint_buffer.back()->GetNextWaypoint().front();
            }
          }

          // Case 2) The buffer already crosses the junction.
          // Check if it is long enough.
          else {

            // Backwards search for the safe point 
            for (auto j = waypoint_buffer.size(); j > 1; --j){
              if (waypoint_buffer.at(j-1)->CheckJunction()){
                safe_point = waypoint_buffer.at(j);
                break;
              }
            }    

            // Keep adding waypoints until the actor has enough space
            if (safe_point->Distance(waypoint_buffer.back()->GetLocation()) < safe_distance){

              final_point = waypoint_buffer.back()->GetNextWaypoint().front();
              while (safe_point->Distance(final_point->GetLocation()) < safe_distance){
                if(waypoint_buffer.back()->CheckJunction()){
                  extend_buffer = true;
                  break;
                }
                waypoint_buffer.push_back(final_point);
                final_point = waypoint_buffer.back()->GetNextWaypoint().front();
              }
            }

            // Backwards search for the final point
            else{
              final_point = waypoint_buffer.back();
              for (auto j = waypoint_buffer.size(); j > 1; --j){
                if (safe_point->Distance(waypoint_buffer.at(j-1)->GetLocation()) < safe_distance){
                  final_point = waypoint_buffer.at(j);
                  break;
                }
              }
            }
          }
        }
        if (final_point == nullptr){
          std::cout << "Final point is nullptr, something failed\n";
        }
        final_points[actor_id] = final_point;
        
        
      }
      // Editing output frames.
      LocalizationToPlannerData &planner_message = current_planner_frame->at(i);
      planner_message.actor = vehicle;
      planner_message.deviation = dot_product;
      planner_message.distance = distance;
      planner_message.approaching_true_junction = approaching_junction;

      // Reading current messenger state of the collision stage before modifying it's frame.
      if ((collision_messenger->GetState() != collision_messenger_state) &&
          !collision_frame_ready) {

        LocalizationToCollisionData &collision_message = current_collision_frame->at(i);
        collision_message.actor = vehicle;
        collision_message.buffer = waypoint_buffer;
        collision_message.approaching_true_junction = approaching_junction;
        collision_message.final_bbox_point = final_points[actor_id];   
      }

      LocalizationToTrafficLightData &traffic_light_message = current_traffic_light_frame->at(i);
      traffic_light_message.actor = vehicle;
      traffic_light_message.closest_waypoint = waypoint_buffer.front();
      traffic_light_message.junction_look_ahead_waypoint = waypoint_buffer.at(look_ahead_index);
    }

    if ((collision_messenger->GetState() != collision_messenger_state) &&
        !collision_frame_ready) {
      collision_frame_ready = true;
    }

  }

  void LocalizationStage::DataReceiver() {

    // Building a list of registered actors and
    // connecting the vehicle ids to their position indices on data arrays.

    if (registered_actors_state != registered_actors.GetState()) {

      actor_list = registered_actors.GetList();

      uint index = 0u;
      for (auto &actor: actor_list) {

        vehicle_id_to_index.insert({actor->GetId(), index});
        ++index;
      }

      registered_actors_state = registered_actors.GetState();
    }

    // Allocating new containers for the changed number of registered vehicles.
    if (number_of_vehicles != actor_list.size()) {

      number_of_vehicles = static_cast<uint>(actor_list.size());
      // Allocating the buffer lists.
      buffer_list = std::make_shared<BufferList>(number_of_vehicles);
      // Allocating output frames to be shared with the motion planner stage.
      planner_frame_a = std::make_shared<LocalizationToPlannerFrame>(number_of_vehicles);
      planner_frame_b = std::make_shared<LocalizationToPlannerFrame>(number_of_vehicles);
      // Allocating output frames to be shared with the collision stage.
      collision_frame_a = std::make_shared<LocalizationToCollisionFrame>(number_of_vehicles);
      collision_frame_b = std::make_shared<LocalizationToCollisionFrame>(number_of_vehicles);
      // Allocating output frames to be shared with the traffic light stage
      traffic_light_frame_a = std::make_shared<LocalizationToTrafficLightFrame>(number_of_vehicles);
      traffic_light_frame_b = std::make_shared<LocalizationToTrafficLightFrame>(number_of_vehicles);
    }

  }

  void LocalizationStage::DataSender() {

    // Since send/receive calls on messenger objects can block if the other
    // end hasn't received/sent data, choose to block on only those stages
    // which takes the most priority (which needs the highest rate of data feed)
    // to run the system well.

    DataPacket<std::shared_ptr<LocalizationToPlannerFrame>> planner_data_packet = {
      planner_messenger_state,
      planner_frame_selector ? planner_frame_a : planner_frame_b
    };
    planner_frame_selector = !planner_frame_selector;
    planner_messenger_state = planner_messenger->SendData(planner_data_packet);

    // Send data to collision stage only if it has finished
    // processing, received the previous message and started processing it.
    int collision_messenger_current_state = collision_messenger->GetState();
    if ((collision_messenger_current_state != collision_messenger_state) &&
        collision_frame_ready) {

      DataPacket<std::shared_ptr<LocalizationToCollisionFrame>> collision_data_packet = {
        collision_messenger_state,
        collision_frame_selector ? collision_frame_a : collision_frame_b
      };

      collision_messenger_state = collision_messenger->SendData(collision_data_packet);
      collision_frame_selector = !collision_frame_selector;
      collision_frame_ready = false;
    }

    // Send data to traffic light stage only if it has finished
    // processing, received the previous message and started processing it.
    int traffic_light_messenger_current_state = traffic_light_messenger->GetState();
    if (traffic_light_messenger_current_state != traffic_light_messenger_state) {
      DataPacket<std::shared_ptr<LocalizationToTrafficLightFrame>> traffic_light_data_packet = {
        traffic_light_messenger_state,
        traffic_light_frame_selector ? traffic_light_frame_a : traffic_light_frame_b
      };

      traffic_light_messenger_state = traffic_light_messenger->SendData(traffic_light_data_packet);
      traffic_light_frame_selector = !traffic_light_frame_selector;
    }
  }

  void LocalizationStage::DrawBuffer(Buffer &buffer) {

    for (uint i = 0u; i < buffer.size() && i < 5; ++i) {
      debug_helper.DrawPoint(buffer.at(i)->GetLocation(), 0.1f, {255u, 0u, 0u}, 0.5f);
    }
  }
}
