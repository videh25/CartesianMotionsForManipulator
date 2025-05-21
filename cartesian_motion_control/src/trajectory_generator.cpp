#include "cartesian_motion_control/trajectory_generator.hpp"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/unsupported/Eigen/src/Splines/Spline.h>
#include <memory>
#include <rclcpp/detail/resolve_use_intra_process.hpp>
#include <stdexcept>
#include <vector>

namespace cartesian_motion_control {

std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
TrajectoryGenerator::generate_spline(
    geometry_msgs::msg::PoseArray::ConstSharedPtr const pose_array) {
  int num_waypoints = pose_array->poses.size();

  std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      return_trajectory = std::make_shared<
          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>();

  if (dt == 0.0) {
    throw std::runtime_error(
        "Sampling time interval not set before generating spline");
  }

  if (num_waypoints == 2) {
    // Use lerp
    const Eigen::Vector3d a(pose_array->poses[0].position.x,
                            pose_array->poses[0].position.y,
                            pose_array->poses[0].position.z);
    const Eigen::Vector3d b(pose_array->poses[1].position.x,
                            pose_array->poses[1].position.y,
                            pose_array->poses[1].position.z);

    const float distance = (b - a).norm();
    const Eigen::Vector3d direction = (b - a) * 1 / distance;
    const float constant_speed_distance_test = distance - 2 * ramp_distance;

    float max_speed, final_ramp_distance, constant_speed_distance;
    if (constant_speed_distance_test <= 0.0) {
      // ramp up; ramp down
      final_ramp_distance = distance / 2;
      max_speed = std::sqrt(acceleration_max * distance);
      constant_speed_distance = 0.0;
    } else {
      // ramp up; const; ramp down
      final_ramp_distance = ramp_distance;
      max_speed = end_effector_speed;
      constant_speed_distance = constant_speed_distance_test;
    }

    float time_ramp = max_speed / acceleration_max;
    float time_constant = constant_speed_distance / max_speed;

    float t = 0.0; // Time to iterate over
    float s = 0.0; // Current distance from start of end-effector
    float v = 0.0; // Current distance from start of end-effector
    std::pair<Eigen::Vector3d, Eigen::Vector3d> position_velocity_pair;
    float this_portion_t = 0.0;

    for (; t < time_ramp; t += dt) {
      this_portion_t = t;
      s = 0.5 * acceleration_max * this_portion_t * this_portion_t;
      v = acceleration_max * this_portion_t;

      position_velocity_pair.first = a + direction * s;
      position_velocity_pair.second = v * direction;
      return_trajectory->push_back(position_velocity_pair);
    }

    for (; t < time_ramp + time_constant; t += dt) {
      this_portion_t = t - time_ramp;
      s = final_ramp_distance + this_portion_t * max_speed;
      v = max_speed;

      position_velocity_pair.first = a + direction * s;
      position_velocity_pair.second = v * direction;
      return_trajectory->push_back(position_velocity_pair);
    }

    for (; t <= 2 * time_ramp + time_constant; t += dt) {
      this_portion_t = t - time_ramp - time_constant;
      s = final_ramp_distance + constant_speed_distance +
          (max_speed * this_portion_t -
           0.5 * acceleration_max * this_portion_t * this_portion_t);
      v = max_speed - acceleration_max * this_portion_t;

      position_velocity_pair.first = a + direction * s;
      position_velocity_pair.second = v * direction;
      return_trajectory->push_back(position_velocity_pair);
    }

  } else if (num_waypoints == 3) {
    // Use quadratic bezier/polynomial
  } else if (num_waypoints > 3) {
    // Use cubic bezier
  } else { // 1 point
    throw std::invalid_argument(
        "One waypoint passed in the pose_array. Need 2 or "
        "more to generate trajectory");
  }

  return return_trajectory;
}

void TrajectoryGenerator::set_max_speed(const float &end_effector_speed_) {
  end_effector_speed = end_effector_speed_;
  recalculate_ramp_distance();
  return;
}

void TrajectoryGenerator::set_acceleration_max(const float &acceleration_max_) {
  acceleration_max = acceleration_max_;
  recalculate_ramp_distance();
  return;
}

void TrajectoryGenerator::recalculate_ramp_distance() {
  if (acceleration_max) {
    // Assuming the manipulator is stationary at start and end
    ramp_distance =
        end_effector_speed * end_effector_speed / (2 * acceleration_max);
  }
}

void TrajectoryGenerator::set_sampling_dt(const float dt_) { dt = dt_; }

} // namespace cartesian_motion_control
