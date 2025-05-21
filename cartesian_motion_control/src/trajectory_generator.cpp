#include "cartesian_motion_control/trajectory_generator.hpp"
#include <kdl/frames.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

namespace cartesian_motion_control {

TrajectoryGenerator::TrajectoryGenerator(const std::string &urdf_path,
                                         const std::string &chain_start_link,
                                         const std::string &chain_end_link) {
  urdf::Model model;
  model.initFile(urdf_path);
  kdl_parser::treeFromUrdfModel(model, kdl_tree);

  KDL::Chain ur10_chain;
  kdl_tree.getChain(chain_start_link, chain_end_link, robot_chain);

  // Default joint limits of -pi to +pi
  unsigned int nj = ur10_chain.getNrOfJoints();
  KDL::JntArray q_min(nj), q_max(nj);
  for (u_int i = 0; i < nj; i++) {
    q_min(i) = -M_PI;
    q_max(i) = M_PI;
  }

  fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
  ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(robot_chain);
  ik_pos_solver = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
      robot_chain, q_min, q_max, *fk_solver, *ik_vel_solver, 100, 1e-6);
  // Solvers initialised

  for (const auto &joint_pair : model.joints_) {
    const urdf::JointSharedPtr &joint = joint_pair.second;

    // Filter fixed joints
    if (joint->type != urdf::Joint::FIXED) {
      joint_names.push_back(joint->name);
    }
  }
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
TrajectoryGenerator::convertCartesianTrajToJointSpace(
    const std::shared_ptr<KDL::Trajectory_Segment> &cartesian_trajectory,
    const float sampling_dt) {

  double duration = cartesian_trajectory->Duration();
  std::cout << "Trajectory duration: " << duration << " seconds" << std::endl;

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> final_joint_trajectory;
  final_joint_trajectory->joint_names = joint_names;

  KDL::Frame pos;
  KDL::Twist twi;
  KDL::JntArray q_last(robot_chain.getNrOfJoints());
  KDL::JntArray q_this(robot_chain.getNrOfJoints());
  KDL::JntArray q_vel_this(robot_chain.getNrOfJoints());
  for (u_int i = 0; i < robot_chain.getNrOfJoints(); i++) {
    q_last(i) = 0.0;
    q_this(i) = 0.0;
    q_vel_this(i) = 0.0;
  }

  trajectory_msgs::msg::JointTrajectoryPoint jt_point_this;
  for (double t = 0; t <= duration; t += sampling_dt) {
    pos = cartesian_trajectory->Pos(t);
    twi = cartesian_trajectory->Vel(t);

    ik_pos_solver->CartToJnt(q_last, pos, q_this);
    ik_vel_solver->CartToJnt(q_this, twi, q_vel_this);

    jt_point_this.positions.clear();
    jt_point_this.velocities.clear();
    for (u_int i = 0; i < robot_chain.getNrOfJoints(); i++) {
      jt_point_this.positions.push_back(q_this(i));
      jt_point_this.velocities.push_back(q_vel_this(i));
    }
    jt_point_this.time_from_start = rclcpp::Duration::from_seconds(t);

    final_joint_trajectory->points.push_back(jt_point_this);

    // Maintain the last joint solution for chain solving
    q_last = q_this;
  }

  return final_joint_trajectory;
}

std::shared_ptr<KDL::Trajectory_Segment>
TrajectoryGenerator::generateCartesianTrajectory(
    const geometry_msgs::msg::PoseArray &waypoints, const float &max_speed,
    const float &max_acc) {
  std::shared_ptr<KDL::Path_RoundedComposite> path =
      std::make_shared<KDL::Path_RoundedComposite>(
          0.1, 0.001, new KDL::RotationalInterpolation_SingleAxis());

  for (const geometry_msgs::msg::Pose &pose : waypoints.poses) {
    KDL::Frame frame(
        KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.z),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
    path->Add(frame);
  }
  path->Finish();

  std::shared_ptr<KDL::VelocityProfile_Trap> vel_profile =
      std::make_shared<KDL::VelocityProfile_Trap>();
  vel_profile->SetMax(max_speed, max_acc);
  vel_profile->SetProfile(0.0, path->PathLength());

  std::shared_ptr<KDL::Trajectory_Segment> trajectory =
      std::make_shared<KDL::Trajectory_Segment>(path.get(), vel_profile.get());

  return trajectory;
}

} // namespace cartesian_motion_control
