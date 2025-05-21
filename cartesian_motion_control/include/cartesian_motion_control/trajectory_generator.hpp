#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>

#include <string>

namespace cartesian_motion_control {

class TrajectoryGenerator {
public:
  TrajectoryGenerator(const std::string &urdf_path,
                      const std::string &chain_start_link,
                      const std::string &chain_end_link);

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
  convertCartesianTrajToJointSpace(
      const std::shared_ptr<KDL::Trajectory_Segment> &cartesian_trajectory,
      const float sampling_dt);

protected:
  KDL::Tree kdl_tree;
  KDL::Chain robot_chain;
  std::vector<std::string> joint_names;

  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver;

}; // namespace cartesian_motion_control

} // namespace cartesian_motion_control
#endif
