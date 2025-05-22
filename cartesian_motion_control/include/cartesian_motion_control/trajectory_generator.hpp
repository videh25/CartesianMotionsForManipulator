#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include "cartesian_motion_control/ik_solver_handlers.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>

#include <string>

namespace cartesian_motion_control {

class TrajectoryGenerator {
public:
  TrajectoryGenerator(
      const std::shared_ptr<IkSolverHandlerInterface> &ik_solver_handler_,
      const std::vector<std::string> &joints_list_);

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
  convertCartesianTrajToJointSpace(
      const std::shared_ptr<KDL::Trajectory_Segment> &cartesian_trajectory,
      const float &sampling_dt, const KDL::JntArray &q_init);
  std::shared_ptr<KDL::Trajectory_Segment>
  generateCartesianTrajectory(const geometry_msgs::msg::PoseArray &waypoints,
                              const float &max_speed, const float &max_acc);

protected:
  std::vector<std::string> joint_names;

  std::shared_ptr<IkSolverHandlerInterface> ik_solver_handler;

}; // namespace cartesian_motion_control

} // namespace cartesian_motion_control
#endif
