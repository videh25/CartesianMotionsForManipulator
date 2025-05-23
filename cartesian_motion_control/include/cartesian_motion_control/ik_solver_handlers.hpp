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
#include <sys/types.h>
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
      q_min(i) = -M_PI * 2;
      q_max(i) = M_PI * 2;
    }

    fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
    ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(robot_chain);
    ik_pos_solver = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
        robot_chain, q_min, q_max, *fk_solver, *ik_vel_solver, 1e8, 1e-3);

    std::vector<std::string> joint_names;
    std::cout << "KDL Parsed " << robot_chain.getNrOfJoints() << " joints"
              << std::endl;

    std::cout << "KDL Chain Structure:\n";
    for (unsigned int i = 0; i < robot_chain.getNrOfSegments(); ++i) {
      const KDL::Segment &segment = robot_chain.getSegment(i);
      const KDL::Joint &joint = segment.getJoint();

      std::string joint_type;
      switch (joint.getType()) {
      case KDL::Joint::None:
        joint_type = "Fixed";
        break;
      case KDL::Joint::RotX:
        joint_type = "RotX";
        break;
      case KDL::Joint::RotY:
        joint_type = "RotY";
        break;
      case KDL::Joint::RotZ:
        joint_type = "RotZ";
        break;
      case KDL::Joint::TransX:
        joint_type = "TransX";
        break;
      case KDL::Joint::TransY:
        joint_type = "TransY";
        break;
      case KDL::Joint::TransZ:
        joint_type = "TransZ";
        break;
      default:
        joint_type = "Other";
        break;
      }

      std::cout << "Segment: " << segment.getName()
                << ", Joint: " << joint.getName() << ", Type: " << joint_type
                << "\n";
    }
    return true;
  }

  bool ConvertToJointSpaceOnce(const KDL::Frame &this_pos,
                               const KDL::Twist &this_twist,
                               const KDL::JntArray &q_last,
                               KDL::JntArray &q_this,
                               KDL::JntArray &q_vel_this) override {
    KDL::Frame seed_ee_pos;
    fk_solver->JntToCart(q_last, seed_ee_pos);
    std::cout << "Seed EE Position: (" << seed_ee_pos.p.x() << ", "
              << seed_ee_pos.p.y() << ", " << seed_ee_pos.p.z() << ")"
              << std::endl;
    std::cout << "Seed EE to current diff: "
              << (seed_ee_pos.p - this_pos.p).Norm() << std::endl;

    int pos_success = ik_pos_solver->CartToJnt(q_last, this_pos, q_this);
    int vel_success = ik_vel_solver->CartToJnt(q_this, this_twist, q_vel_this);

    switch (pos_success) {
    case KDL::ChainIkSolverPos_NR_JL::E_MAX_ITERATIONS_EXCEEDED:
      std::cout << "IK failed to calculate position: Max iterations exceeded"
                << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverPos_NR_JL::E_NOT_UP_TO_DATE:
      std::cout << "IK failed to calculate position: Internal data outdated"
                << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverPos_NR_JL::E_SIZE_MISMATCH:
      std::cout << "IK failed to calculate position: Size mismatched"
                << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverPos_NR_JL::E_NOERROR:
      std::cout << "IK Successful" << std::endl;
      break;
    }

    switch (vel_success) {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
      std::cout << "IK failed to calculate velocity: pinv singular"
                << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverVel_pinv::E_SVD_FAILED:
      std::cout << "IK failed to calculate velocity: SVD failed" << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverVel_pinv::E_NOERROR:
      std::cout << "IK velocity Successful" << std::endl;
      break;
    }
    return true;

    // Check if ik is working correctly
    // KDL::Frame test_frame;
    // fk_solver->JntToCart(q_this, test_frame);
    // if (!KDL::Equal(test_frame.p, this_pos.p, 1e-4)) {
    //   std::cout << "IK Failure detected in frame vector" << std::endl;
    // }
    // if (!KDL::Equal(test_frame.M, this_pos.M, 1e-4)) {
    // std::cout << "IK Failure detected in frame rotation" << std::endl;
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
      q_min(i) = -M_PI * 2;
      q_max(i) = M_PI * 2;
    }

    fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
    ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(robot_chain);
    ik_pos_solver = std::make_shared<TRAC_IK::TRAC_IK>(
        robot_chain, q_min, q_max, 0.01, 1e-2, TRAC_IK::SolveType::Distance);

    std::vector<std::string> joint_names;
    std::cout << "KDL Parsed " << robot_chain.getNrOfJoints() << " joints"
              << std::endl;
    std::cout << "KDL Parsed " << robot_chain.getNrOfJoints() << " joints"
              << std::endl;

    std::cout << "KDL Chain Structure:\n";
    for (unsigned int i = 0; i < robot_chain.getNrOfSegments(); ++i) {
      const KDL::Segment &segment = robot_chain.getSegment(i);
      const KDL::Joint &joint = segment.getJoint();

      std::string joint_type;
      switch (joint.getType()) {
      case KDL::Joint::None:
        joint_type = "Fixed";
        break;
      case KDL::Joint::RotX:
        joint_type = "RotX";
        break;
      case KDL::Joint::RotY:
        joint_type = "RotY";
        break;
      case KDL::Joint::RotZ:
        joint_type = "RotZ";
        break;
      case KDL::Joint::TransX:
        joint_type = "TransX";
        break;
      case KDL::Joint::TransY:
        joint_type = "TransY";
        break;
      case KDL::Joint::TransZ:
        joint_type = "TransZ";
        break;
      default:
        joint_type = "Other";
        break;
      }

      std::cout << "Segment: " << segment.getName()
                << ", Joint: " << joint.getName() << ", Type: " << joint_type
                << "\n";
    }
    return true;
  }

  bool ConvertToJointSpaceOnce(const KDL::Frame &this_pos,
                               const KDL::Twist &this_twist,
                               const KDL::JntArray &q_last,
                               KDL::JntArray &q_this,
                               KDL::JntArray &q_vel_this) override {
    KDL::Twist tol;
    tol = KDL::Twist(KDL::Vector(0.1, 0.1, 0.1), KDL::Vector(0.1, 0.1, 0.1));

    KDL::Frame seed_ee_pos;
    fk_solver->JntToCart(q_last, seed_ee_pos);
    // std::cout << "Seed EE Position: (" << seed_ee_pos.p.x() << ", "
    //           << seed_ee_pos.p.y() << ", " << seed_ee_pos.p.z() << ")"
    //           << std::endl;
    // std::cout << "Seed EE to current diff: "
    //           << (seed_ee_pos.p - this_pos.p).Norm() << std::endl;

    int pos_success = ik_pos_solver->CartToJnt(q_last, this_pos, q_this, tol);
    int vel_success = ik_vel_solver->CartToJnt(q_this, this_twist, q_vel_this);

    if (pos_success <= 0) {
      std::cout << "IK failed to calculate position" << std::endl;
      return false;
    } else {
      std::cout << "IK position successful" << std::endl;
    }

    switch (vel_success) {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
      std::cout << "IK failed to calculate velocity: pinv singular"
                << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverVel_pinv::E_SVD_FAILED:
      std::cout << "IK failed to calculate velocity: SVD failed" << std::endl;
      return false;
      break;
    case KDL::ChainIkSolverVel_pinv::E_NOERROR:
      std::cout << "IK velocity Successful" << std::endl;
      break;
    }
    // Check if ik is working correctly
    // KDL::Frame test_frame;
    // fk_solver->JntToCart(q_this, test_frame);
    // if (!KDL::Equal(test_frame.p, this_pos.p, 1e-4)) {
    //   std::cout << "IK Failure detected in frame vector" << std::endl;
    // }
    // if (!KDL::Equal(test_frame.M, this_pos.M, 1e-4)) {
    //   std::cout << "IK Failure detected in frame rotation" << std::endl;
    // }
    // std::cout << "IK Solution for (" << this_pos.p.x() << ", " <<
    // this_pos.p.y()
    //           << ", " << this_pos.p.z() << ") is :::" << std::endl;
    // std::cout << "(";
    // for (u_int i = 0; i < 6; i++) {
    //   std::cout << q_this(i) << ", ";
    // }
    // std::cout << std::endl;
    return true;
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
