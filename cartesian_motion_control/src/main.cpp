#include "cartesian_motion_control/cartesian_motion_server.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<cartesian_motion_control::CartesianTrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}
