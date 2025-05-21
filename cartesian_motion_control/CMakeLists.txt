cmake_minimum_required(VERSION 3.8)
project(cartesian_motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cartesian_motion_interfaces REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

add_library(cartesian_trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(cartesian_trajectory_generator rclcpp geometry_msgs trajectory_msgs urdf kdl_parser orocos_kdl Eigen3)
target_include_directories(cartesian_trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(cartesian_motion_server src/cartesian_motion_server.cpp)
ament_target_dependencies(cartesian_motion_server cartesian_motion_interfaces rclcpp rclcpp_action trajectory_msgs geometry_msgs)
target_include_directories(cartesian_motion_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cartesian_motion_server cartesian_trajectory_generator)

add_executable(cartesian_motion_control_node src/main.cpp)
ament_target_dependencies(cartesian_motion_control_node cartesian_motion_interfaces urdf kdl_parser orocos_kdl Eigen3)
target_include_directories(cartesian_motion_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cartesian_motion_control_node cartesian_motion_server)

install(TARGETS
  cartesian_motion_server
  cartesian_trajectory_generator
  cartesian_motion_control_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
