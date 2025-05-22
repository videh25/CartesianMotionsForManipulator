#ifndef __IK_SOVLER_HANDLER__
#define __IK_SOVLER_HANDLER__

#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <urdf/model.h>

namespace cartesian_motion_control {

class IkSolverHandlerInterface {
public:
  IkSolverHandlerInterface(const std::string &urdf_path_,
                           const std::string &chain_start_link_,
                           const std::string &chain_end_link_)
      : urdf_path(urdf_path_), chain_start_link(chain_start_link_),
        chain_end_link(chain_end_link_) {}

  virtual u_int GetNumJoints() = 0;
  virtual bool Configure() = 0;
  virtual bool ConvertToJointSpaceOnce(const KDL::Frame &this_pos,
                                       const KDL::Twist &this_twist,
                                       const KDL::JntArray &q_last,
                                       KDL::JntArray &q_this,
                                       KDL::JntArray &q_vel_this) = 0;

protected:
  std::string urdf_path;
  std::string chain_start_link;
  std::string chain_end_link;
};

class KDLSolverHandler : public IkSolverHandlerInterface {
public:
  KDLSolverHandler(const std::string &urdf_path_,
                   const std::string &chain_start_link_,
                   const std::string &chain_end_link_)
      : IkSolverHandlerInterface(urdf_path_, chain_start_link_,
                                 chain_end_link_) {}

  u_int GetNumJoints() override { return robot_chain.getNrOfJoints(); }

  bool Configure() override {
    urdf::Model model;
    model.initFile(urdf_path);
    kdl_parser::treeFromUrdfModel(model, kdl_tree);

    kdl_tree.getChain(chain_start_link, chain_end_link, robot_chain);

    // Default joint limits of -pi to +pi
    unsigned int nj = robot_chain.getNrOfJoints();
    KDL::JntArray q_min(nj), q_max(nj);
    for (u_int i = 0; i < nj; i++) {
      q_min(i) = -M_PI;
      q_max(i) = M_PI;
    }

    fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
    ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(robot_chain);
    ik_pos_solver = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
        robot_chain, q_min, q_max, *fk_solver, *ik_vel_solver, 200, 1e-4);

    std::vector<std::string> joint_names;
    std::cout << "KDL Parsed " << robot_chain.getNrOfJoints() << " joints"
              << std::endl;
    std::cout << "Joint Names: [";
    for (unsigned int i = 0; i < robot_chain.getNrOfSegments(); ++i) {
      const KDL::Joint &joint = robot_chain.getSegment(i).getJoint();
      KDL::Joint::JointType jtype = joint.getType();
      if ((jtype == KDL::Joint::JointType::RotAxis) ||
          (jtype == KDL::Joint::JointType::RotX) ||
          (jtype == KDL::Joint::JointType::RotY) ||
          (jtype == KDL::Joint::JointType::RotZ)) {
        std::cout << joint.getName() << ", ";
      }
    }
    std::cout << "]" << std::endl;
    return true;
  }

  bool ConvertToJointSpaceOnce(const KDL::Frame &this_pos,
                               const KDL::Twist &this_twist,
                               const KDL::JntArray &q_last,
                               KDL::JntArray &q_this,
                               KDL::JntArray &q_vel_this) override {

    int pos_success = ik_pos_solver->CartToJnt(q_last, this_pos, q_this);
    int vel_success = ik_vel_solver->CartToJnt(q_this, this_twist, q_vel_this);

    switch (pos_success) {
    case KDL::ChainIkSolverPos_NR_JL::E_MAX_ITERATIONS_EXCEEDED:
      std::cout << "IK failed to calculate position: Max iterations exceeded"
                << std::endl;
      break;
    case KDL::ChainIkSolverPos_NR_JL::E_NOT_UP_TO_DATE:
      std::cout << "IK failed to calculate position: Internal data outdated"
                << std::endl;
      break;
    case KDL::ChainIkSolverPos_NR_JL::E_SIZE_MISMATCH:
      std::cout << "IK failed to calculate position: Size mismatched"
                << std::endl;
      break;
    case KDL::ChainIkSolverPos_NR_JL::E_NOERROR:
      std::cout << "IK Successful" << std::endl;
      return true;
      break;
    }

    switch (vel_success) {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
      std::cout << "IK failed to calculate velocity: pinv singular"
                << std::endl;
      break;
    case KDL::ChainIkSolverVel_pinv::E_SVD_FAILED:
      std::cout << "IK failed to calculate velocity: SVD failed" << std::endl;
      break;
    case KDL::ChainIkSolverVel_pinv::E_NOERROR:
      std::cout << "IK velocity Successful" << std::endl;
      return true;
      break;
    }
    return false;

    // Check if ik is working correctly
    // KDL::Frame test_frame;
    // fk_solver->JntToCart(q_this, test_frame);
    // if (!KDL::Equal(test_frame.p, this_pos.p, 1e-4)) {
    //   std::cout << "IK Failure detected in frame vector" << std::endl;
    // }
    // if (!KDL::Equal(test_frame.M, this_pos.M, 1e-4)) {
    //   std::cout << "IK Failure detected in frame rotation" << std::endl;
    // }
  }

protected:
  KDL::Tree kdl_tree;
  KDL::Chain robot_chain;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver;
};

class TracIkSolverHandler : public IkSolverHandlerInterface {
public:
  TracIkSolverHandler(const std::string &urdf_path_,
                      const std::string &chain_start_link_,
                      const std::string &chain_end_link_)
      : IkSolverHandlerInterface(urdf_path_, chain_start_link_,
                                 chain_end_link_) {}

  u_int GetNumJoints() override { return robot_chain.getNrOfJoints(); }

  bool Configure() override {
    urdf::Model model;
    model.initFile(urdf_path);
    kdl_parser::treeFromUrdfModel(model, kdl_tree);

    kdl_tree.getChain(chain_start_link, chain_end_link, robot_chain);

    // Default joint limits of -pi to +pi
    unsigned int nj = robot_chain.getNrOfJoints();
    KDL::JntArray q_min(nj), q_max(nj);
    for (u_int i = 0; i < nj; i++) {
      q_min(i) = -M_PI;
      q_max(i) = M_PI;
    }

    fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
    ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(robot_chain);
    ik_pos_solver =
        std::make_shared<TRAC_IK::TRAC_IK>(robot_chain, q_min, q_max, 1e-4);

    std::vector<std::string> joint_names;
    std::cout << "KDL Parsed " << robot_chain.getNrOfJoints() << " joints"
              << std::endl;
    std::cout << "Joint Names: [";
    for (unsigned int i = 0; i < robot_chain.getNrOfSegments(); ++i) {
      const KDL::Joint &joint = robot_chain.getSegment(i).getJoint();
      KDL::Joint::JointType jtype = joint.getType();
      if ((jtype == KDL::Joint::JointType::RotAxis) ||
          (jtype == KDL::Joint::JointType::RotX) ||
          (jtype == KDL::Joint::JointType::RotY) ||
          (jtype == KDL::Joint::JointType::RotZ)) {
        std::cout << joint.getName() << ", ";
      }
    }
    std::cout << "]" << std::endl;
    return true;
  }

  bool ConvertToJointSpaceOnce(const KDL::Frame &this_pos,
                               const KDL::Twist &this_twist,
                               const KDL::JntArray &q_last,
                               KDL::JntArray &q_this,
                               KDL::JntArray &q_vel_this) override {
    KDL::Twist tol;
    tol = KDL::Twist(KDL::Vector(0.01, 0.01, 0.01), KDL::Vector(0.1, 0.1, 0.1));

    ik_pos_solver->CartToJnt(q_last, this_pos, q_this, tol);
    ik_vel_solver->CartToJnt(q_this, this_twist, q_vel_this);

    // Check if ik is working correctly
    KDL::Frame test_frame;
    fk_solver->JntToCart(q_this, test_frame);
    if (!KDL::Equal(test_frame.p, this_pos.p, 1e-4)) {
      std::cout << "IK Failure detected in frame vector" << std::endl;
    }
    if (!KDL::Equal(test_frame.M, this_pos.M, 1e-4)) {
      std::cout << "IK Failure detected in frame rotation" << std::endl;
    }
  }

protected:
  KDL::Tree kdl_tree;
  KDL::Chain robot_chain;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
  std::shared_ptr<TRAC_IK::TRAC_IK> ik_pos_solver;
};
} // namespace cartesian_motion_control
#endif
