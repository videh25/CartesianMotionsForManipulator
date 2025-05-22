#include "cartesian_motion_control/trajectory_generator.hpp"
#include <cmath>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace cartesian_motion_control {

TrajectoryGenerator::TrajectoryGenerator(
    const std::shared_ptr<IkSolverHandlerInterface> &ik_solver_handler_,
    const std::vector<std::string> &joints_list_)
    : ik_solver_handler(ik_solver_handler_) {
  ik_solver_handler->Configure();
  joint_names = joints_list_;
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
TrajectoryGenerator::convertCartesianTrajToJointSpace(
    const std::shared_ptr<KDL::Trajectory_Segment> &cartesian_trajectory,
    const float &sampling_dt, const KDL::JntArray &q_init) {

  double duration = cartesian_trajectory->Duration();
  u_int nj = ik_solver_handler->GetNumJoints();

  std::cout << "Trajectory duration: " << duration << " seconds" << std::endl;

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
      final_joint_trajectory =
          std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  final_joint_trajectory->joint_names = joint_names;

  KDL::Frame pos;
  KDL::Twist twi;
  KDL::JntArray q_last(nj);
  KDL::JntArray q_this(nj);
  KDL::JntArray q_vel_this(nj);
  KDL::JntArray q_vel_last(nj);
  trajectory_msgs::msg::JointTrajectoryPoint jt_point_this;

  for (u_int i = 0; i < nj; i++) {
    q_last(i) = q_init(i);
    q_this(i) = 0.0;
    q_vel_this(i) = 0.0;
    jt_point_this.positions.push_back(0);
    jt_point_this.velocities.push_back(0);
  }

  for (double t = 0; t <= duration; t += sampling_dt) {
    pos = cartesian_trajectory->Pos(t);
    twi = cartesian_trajectory->Vel(t);

    // Convert this pose to cspace
    bool ik_success = ik_solver_handler->ConvertToJointSpaceOnce(
        pos, twi, q_last, q_this, q_vel_this);

    if (!ik_success) {
      // IK Failed once and hence stopping
      final_joint_trajectory.reset();
      return final_joint_trajectory;
    }

    // Store in the trajectory msg
    for (u_int i = 0; i < nj; i++) {
      jt_point_this.positions[i] = q_this(i);
      jt_point_this.velocities[i] = q_vel_this(i);
    }
    jt_point_this.time_from_start = rclcpp::Duration::from_seconds(t + 5);
    final_joint_trajectory->points.push_back(jt_point_this);

    {
      // Check for joint state continuity
      double diff_norm = 0, vel_diff_norm = 0;
      for (u_int i = 0; i < nj; i++) {
        diff_norm += (q_this(i) - q_last(i)) * (q_this(i) - q_last(i));
        vel_diff_norm +=
            (q_vel_this(i) - q_vel_last(i)) * (q_vel_this(i) - q_vel_last(i));
      }
      diff_norm = std::sqrt(diff_norm);
      vel_diff_norm = std::sqrt(vel_diff_norm);

      if (diff_norm > 0.1) {
        std::cout << "q tolerance violated at t = " << t << " with value "
                  << diff_norm << std::endl;
      }
      if (vel_diff_norm > 0.1) {
        std::cout << "q_vel tolerance violated at t = " << t << " with value "
                  << vel_diff_norm << std::endl;
      }
    }

    // Maintain the last joint solution for chain solving
    q_last = q_this;
    q_vel_last = q_vel_this;
  }

  return final_joint_trajectory;
}

std::shared_ptr<KDL::Trajectory_Segment>
TrajectoryGenerator::generateCartesianTrajectory(
    const geometry_msgs::msg::PoseArray &waypoints, const float &max_speed,
    const float &max_acc) {
  KDL::Path_RoundedComposite *path = new KDL::Path_RoundedComposite(
      0.1, 0.001, new KDL::RotationalInterpolation_SingleAxis());

  for (const geometry_msgs::msg::Pose &pose : waypoints.poses) {
    KDL::Frame frame(
        KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.z),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
    path->Add(frame);
  }
  path->Finish();

  KDL::VelocityProfile_Trap *vel_profile = new KDL::VelocityProfile_Trap();
  vel_profile->SetMax(max_speed, max_acc);
  vel_profile->SetProfile(0.0, path->PathLength());

  std::shared_ptr<KDL::Trajectory_Segment> trajectory =
      std::make_shared<KDL::Trajectory_Segment>(path, vel_profile);
  KDL::Frame pos;
  KDL::Twist twi;

  return trajectory;
}

} // namespace cartesian_motion_control
