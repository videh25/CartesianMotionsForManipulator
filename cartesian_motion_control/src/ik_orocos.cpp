#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

using namespace KDL;

int main() {
  // 1. Load URDF model
  urdf::Model model;
  model.initFile("/home/videh22/Documents/catesian_motions_ws/src/"
                 "cartesian_motion_control/urdf/ur10.urdf");

  // 2. Parse into KDL tree
  KDL::Tree my_tree;
  kdl_parser::treeFromUrdfModel(model, my_tree);

  // 3. Extract KDL chain (base_link to tool0)
  KDL::Chain ur10_chain;
  my_tree.getChain("base_link", "tool0", ur10_chain);

  // 4. Define joint limits
  unsigned int nj = ur10_chain.getNrOfJoints();
  KDL::JntArray q_min(nj), q_max(nj);
  q_min(0) = -M_PI;
  q_max(0) = M_PI;
  q_min(1) = -M_PI;
  q_max(1) = M_PI;
  q_min(2) = -M_PI;
  q_max(2) = M_PI;
  q_min(3) = -M_PI;
  q_max(3) = M_PI;
  q_min(4) = -M_PI;
  q_max(4) = M_PI;
  q_min(5) = -M_PI;
  q_max(5) = M_PI;

  // 5. Define solvers
  ChainFkSolverPos_recursive fk_solver(ur10_chain);
  ChainIkSolverVel_pinv ik_solver_vel(ur10_chain);
  ChainIkSolverPos_NR_JL ik_solver(ur10_chain, q_min, q_max, fk_solver,
                                   ik_solver_vel, 100, 1e-6);

  // 6. Desired pose
  KDL::Frame desired_pose =
      KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0.4, 0.2, 0.5));

  // 7. Seed/init guess
  KDL::JntArray q_init(nj);
  for (unsigned int i = 0; i < nj; ++i)
    q_init(i) = 0.0;

  // 8. Result
  KDL::JntArray q_result(nj);
  int status = ik_solver.CartToJnt(q_init, desired_pose, q_result);

  if (status >= 0) {
    std::cout << "IK Solved. Joint angles:\n";
    for (unsigned int i = 0; i < nj; ++i)
      std::cout << "q[" << i << "] = " << q_result(i) << std::endl;
  } else {
    std::cerr << "IK failed with status: " << status << std::endl;
  }

  return 0;
}
