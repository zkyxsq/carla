// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/trafficmanager/PIDController.h"

#include <algorithm>

namespace carla {
namespace traffic_manager {

namespace PIDControllerConstants {

  const float MAX_THROTTLE = 0.7f;
  const float MAX_BRAKE = 1.0f;

  // PID will be stable only over 20 FPS.
  const float dt = 1/20.0f;

} // namespace PIDControllerConstants

  using namespace PIDControllerConstants;

  PIDController::PIDController() {}

  // Initializing present state.
  StateEntry PIDController::StateUpdate(
      StateEntry previous_state,
      float current_velocity,
      float target_velocity,
      float horizontal_velocity,
      float angular_deviation,
      float distance,
      TimeInstance current_time) {

    traffic_manager::StateEntry current_state = {
      angular_deviation, distance,
      (current_velocity - target_velocity) / target_velocity,
      horizontal_velocity,
      current_time,
      0.0f,
      0.0f,
      0.0f
    };

    // Calculating integrals.
    // current_state.deviation_integral = angular_deviation * dt + previous_state.deviation_integral;
    // current_state.distance_integral = distance * dt + previous_state.distance_integral;
    current_state.velocity_integral = dt * current_state.velocity + previous_state.velocity_integral;

    return current_state;
  }

  ActuationSignal PIDController::RunStep(
      StateEntry present_state,
      StateEntry previous_state,
      const std::vector<float> &longitudinal_parameters,
      const std::vector<float> &lateral_parameters,
      const float max_steer_angle) const {

    // Longitudinal PID calculation.
    const float expr_v =
        longitudinal_parameters[0] * present_state.velocity +
        longitudinal_parameters[1] * present_state.velocity_integral +
        longitudinal_parameters[2] * (present_state.velocity -
        previous_state.velocity) / dt;

    float throttle;
    float brake;

    if (expr_v < 0.0f) {
      throttle = std::min(std::abs(expr_v), MAX_THROTTLE);
      brake = 0.0f;
    } else {
      throttle = 0.0f;
      brake = std::min(expr_v, MAX_BRAKE);
    }

    // Lateral (Hoffmann-Stanley) controller calculation.
    float steer_angle = present_state.deviation +
                        std::atan((1.0f + 0*lateral_parameters.front()) * present_state.distance / (present_state.horizontal_velocity + 0.001));
    // Accounting for change of co-ordinate system from carla to unreal by multiplying with -1.
    // Also, converting calculated steer angle to fraction of maximum allowed.
    float steer_actuation_signal = (steer_angle/max_steer_angle);
    steer_actuation_signal = std::max(-0.8f, std::min(steer_actuation_signal, 0.8f));

    return ActuationSignal{throttle, brake, steer_actuation_signal};
  }

} // namespace traffic_manager
} // namespace carla
