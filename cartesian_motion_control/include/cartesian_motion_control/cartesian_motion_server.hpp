#ifndef __CARTESIAN_MOTION_SERVER_HPP__
#define __CARTESIAN_MOTION_SERVER_HPP__

#include "cartesian_motion_control/ik_solver_handlers.hpp"
#include "cartesian_motion_control/trajectory_generator.hpp"
#include <memory>

#include "cartesian_motion_interfaces/action/cartesian_trajectory.hpp"
#include <geometry_msgs/msg/pose_array.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

namespace cartesian_motion_control {
class CartesianTrajectoryServer : public rclcpp::Node {
public:
  explicit CartesianTrajectoryServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<
      cartesian_motion_interfaces::action::CartesianTrajectory>::SharedPtr
      action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<
          const cartesian_motion_interfaces::action::CartesianTrajectory::Goal>
          goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                    cartesian_motion_interfaces::action::CartesianTrajectory>>
                    goal_handle);

  void
  handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                      cartesian_motion_interfaces::action::CartesianTrajectory>>
                      goal_handle);

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   cartesian_motion_interfaces::action::CartesianTrajectory>>
                   goal_handle);

  std::shared_ptr<TrajectoryGenerator> tg_;
  float sampling_dt;
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>>
      joint_trajectory_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      robot_state_subscription;
  std::mutex robot_state_mtx;
  KDL::JntArray last_jnt_state;
}; // class CartesianTrajectoryServer

} // namespace cartesian_motion_control

#endif
