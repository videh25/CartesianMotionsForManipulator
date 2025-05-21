#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/unsupported/Eigen/Splines>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cartesian_motion_control {

class TrajectoryGenerator {
public:
  TrajectoryGenerator(const TrajectoryGenerator &) = delete;
  TrajectoryGenerator &operator=(const TrajectoryGenerator &) = delete;
  TrajectoryGenerator(TrajectoryGenerator &&) = delete;
  TrajectoryGenerator &operator=(TrajectoryGenerator &&) = delete;

  static TrajectoryGenerator &GetInstance() {
    static TrajectoryGenerator
        instance; // Guaranteed to be initialized once in a thread-safe manner
    return instance;
  };

  void set_max_speed(const float &end_effector_speed);
  void set_acceleration_max(const float &acceleration_max_);
  void set_sampling_dt(const float dt_);

  std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
  generate_spline(
      geometry_msgs::msg::PoseArray::ConstSharedPtr const pose_array);

protected:
  float end_effector_speed;
  float acceleration_max;
  float ramp_distance;
  float dt; // Sampling time interval

  void recalculate_ramp_distance();

private:
  TrajectoryGenerator() {};
  ~TrajectoryGenerator() {};
  static TrajectoryGenerator instance;
};

} // namespace cartesian_motion_control

#endif
