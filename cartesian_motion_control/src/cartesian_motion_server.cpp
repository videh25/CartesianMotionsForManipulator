#include "cartesian_motion_control/cartesian_motion_server.hpp"
#include "cartesian_motion_control/ik_solver_handlers.hpp"
#include <cartesian_motion_interfaces/action/detail/cartesian_trajectory__struct.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <mutex>
#include <rclcpp_action/server.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <vector>

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
      joint_trajectory_topic, robot_state_topic;
  std::vector<std::string> joint_names;

  this->declare_parameter<std::string>("urdf_path");
  this->declare_parameter<std::string>("chain_start_link");
  this->declare_parameter<std::string>("chain_end_link");
  this->declare_parameter<double>("sampling_dt");
  this->declare_parameter<std::string>("joint_trajectory_topic");
  this->declare_parameter<std::string>("robot_state_topic");
  this->declare_parameter<std::vector<std::string>>("link_names");

  this->get_parameter("urdf_path", urdf_path);
  this->get_parameter("chain_start_link", chain_start_link);
  this->get_parameter("chain_end_link", chain_end_link);
  this->get_parameter("sampling_dt", sampling_dt);
  this->get_parameter("joint_trajectory_topic", joint_trajectory_topic);
  this->get_parameter("robot_state_topic", robot_state_topic);
  this->get_parameter("link_names", joint_names);

  joint_trajectory_publisher =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          joint_trajectory_topic, 10);

  std::shared_ptr<IkSolverHandlerInterface> ik_solver_handler =
      std::make_shared<KDLSolverHandler>(urdf_path, chain_start_link,
                                         chain_end_link);
  tg_ = std::make_shared<TrajectoryGenerator>(ik_solver_handler, joint_names);
  auto robot_state_callback =
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lg_(robot_state_mtx);
        last_jnt_state.resize(msg->position.size());
        // std::cout << "Updating robot pose to: [";
        for (u_int i = 0; i < msg->position.size(); i++) {
          // std::cout << msg->position[i] << ", ";
          last_jnt_state(i) = msg->position[i];
        }
        // std::cout << "]" << std::endl;
      };
  robot_state_subscription =
      this->create_subscription<sensor_msgs::msg::JointState>(
          robot_state_topic, 1, robot_state_callback);
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

  std::shared_ptr<KDL::Trajectory_Segment> trajectory =
      tg_->generateCartesianTrajectory(goal->waypoints, goal->maximum_speed,
                                       goal->maximum_acceleration);

  feedback->feedback = "Calculated a path with duration of " +
                       std::to_string(trajectory->Duration()) + " secs";
  goal_handle->publish_feedback(feedback);

  robot_state_mtx.lock();
  KDL::JntArray robot_state = last_jnt_state;
  robot_state_mtx.unlock();

  auto joint_trajectory = tg_->convertCartesianTrajToJointSpace(
      trajectory, sampling_dt, robot_state);
  feedback->feedback = "Generated a joint trajectory of " +
                       std::to_string(joint_trajectory->points.size()) +
                       " elemenets";
  goal_handle->publish_feedback(feedback);

  // joint_trajectory->header.stamp = now();
  // joint_trajectory->header.frame_id = "base_link";
  joint_trajectory_publisher->publish(*joint_trajectory);

  feedback->feedback = "Published the joint trajectory!";
  goal_handle->publish_feedback(feedback);

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory executed!");
}
} // namespace cartesian_motion_control
