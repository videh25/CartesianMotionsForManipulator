#include "cartesian_motion_control/cartesian_motion_server.hpp"
#include <cartesian_motion_interfaces/action/detail/cartesian_trajectory__struct.hpp>
#include <rclcpp_action/server.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>

namespace cartesian_motion_control {

CartesianTrajectoryServer::CartesianTrajectoryServer(
    const rclcpp::NodeOptions &options)
    : Node("cartesian_motion_server", options) {
  this->action_server_ = rclcpp_action::create_server<
      cartesian_motion_interfaces::action::CartesianTrajectory>(
      this, "cartesian_motion",
      std::bind(&CartesianTrajectoryServer::handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&CartesianTrajectoryServer::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&CartesianTrajectoryServer::handle_accepted, this,
                std::placeholders::_1));
  std::string urdf_path, chain_start_link, chain_end_link,
      joint_trajectory_topic;
  this->get_parameter("urdf_path", urdf_path);
  this->get_parameter("chain_start_link", chain_start_link);
  this->get_parameter("chain_end_link", chain_end_link);
  this->get_parameter("sampling_dt", sampling_dt);
  this->get_parameter("joint_trajectory_topic", joint_trajectory_topic);

  joint_trajectory_publisher =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          joint_trajectory_topic, 10);
  tg_ = std::make_shared<TrajectoryGenerator>(urdf_path, chain_start_link,
                                              chain_end_link);
}

rclcpp_action::GoalResponse CartesianTrajectoryServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<
        const cartesian_motion_interfaces::action::CartesianTrajectory::Goal>
        goal) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CartesianTrajectoryServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        cartesian_motion_interfaces::action::CartesianTrajectory>>
        goal_handle) {
  return rclcpp_action::CancelResponse::REJECT;
}

void CartesianTrajectoryServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        cartesian_motion_interfaces::action::CartesianTrajectory>>
        goal_handle) {
  std::thread{std::bind(&CartesianTrajectoryServer::execute, this,
                        std::placeholders::_1),
              goal_handle}
      .detach();
}

void CartesianTrajectoryServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        cartesian_motion_interfaces::action::CartesianTrajectory>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Calculating Trajectories");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<
      cartesian_motion_interfaces::action::CartesianTrajectory::Feedback>();

  auto result = std::make_shared<
      cartesian_motion_interfaces::action::CartesianTrajectory::Result>();

  auto trajectory = tg_->generateCartesianTrajectory(
      goal->waypoints, goal->maximum_speed, goal->maximum_acceleration);

  feedback->feedback = "Calculated a path with duration of " +
                       std::to_string(trajectory->Duration()) + " secs";
  goal_handle->publish_feedback(feedback);

  auto joint_trajectory =
      tg_->convertCartesianTrajToJointSpace(trajectory, sampling_dt);
  feedback->feedback = "Generated a joint trajectory of " +
                       std::to_string(joint_trajectory->points.size()) +
                       " elemenets";
  goal_handle->publish_feedback(feedback);

  joint_trajectory_publisher->publish(*joint_trajectory);

  feedback->feedback = "Published the joint trajectory!";
  goal_handle->publish_feedback(feedback);

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory executed!");
}
} // namespace cartesian_motion_control
