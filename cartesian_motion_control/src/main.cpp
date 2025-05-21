// trajectory_example.cpp
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <memory>

int main() {
  // Define start and end frames
  KDL::Frame start(KDL::Vector(0.0, 0.0, 0.0));
  KDL::Frame end(KDL::Vector(1.0, 1.0, 1.0));
  std::shared_ptr<KDL::RotationalInterpolation> orient =
      std::make_shared<KDL::RotationalInterpolation_SingleAxis>();

  // Create a path (straight line)
  double eqradius =
      0.001; // path resolution (used for blending, not relevant for line)
  std::shared_ptr<KDL::Path_Line> path =
      std::make_shared<KDL::Path_Line>(start, end, orient.get(), eqradius);

  // Create a trapezoidal velocity profile
  double max_vel = 1.0; // m/s
  double max_acc = 1.0; // m/s^2
  std::shared_ptr<KDL::VelocityProfile_Trap> velprof =
      std::make_shared<KDL::VelocityProfile_Trap>();
  velprof->SetMax(max_vel, max_acc);
  velprof->SetProfile(0, path->PathLength());

  // Combine path and velocity profile into a trajectory
  std::shared_ptr<KDL::Trajectory_Segment> trajectory =
      std::make_shared<KDL::Trajectory_Segment>(path.get(), velprof.get());

  // Duration of the trajectory
  double duration = trajectory->Duration();
  std::cout << "Trajectory duration: " << duration << " seconds" << std::endl;

  // Sample the trajectory
  KDL::Frame pos;
  KDL::Twist twi;
  for (double t = 0; t <= duration; t += 0.1) {
    pos = trajectory->Pos(t);
    twi = trajectory->Vel(t);
    std::cout << "t = " << t << "s: Position = [" << pos.p.x() << ", "
              << pos.p.y() << ", " << pos.p.z() << "]" << std::endl;
    std::cout << "twist norm: " << twi.vel.Norm() << std::endl;
    // std::cout << "t = " << t << "s: Twist_v = [" << twi.vel.x() << ", "
    //           << twi.vel.y() << ", " << twi.vel.z() << "]" << std::endl;
    // std::cout << "t = " << t << "s: Twist_r = [" << twi.rot.x() << ", "
    //           << twi.rot.y() << ", " << twi.rot.z() << "]" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
  }

  return 0;
}
