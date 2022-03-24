//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include <functional>
#include <algorithm>

#include "../../RaisimGymEnv.hpp"
#include "impedance_control_module.h"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), unifDistZeroOne_(0.0, 1.0), controller_(cfg["controller"]) {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
//    parseURDF();
//    std::ofstream myfile;
//    myfile.open (ros::package::getPath("ros_raisim_interface") + "/resource/temp.urdf", std::ios::out);
//    myfile << *urdf_;
//    myfile.close();
    ouzel_ = world_->addArticulatedSystem(resourceDir_ + "/ouzel/temp.urdf");

//    std::string baseLink = ouzel_->getBodyIdx("ouzel/base_link");
//    ouzel_ = world_->addArticulatedSystem(resourceDir_+"/ouzel/urdf/model.urdf"); //used to be anymal.urdf
    ouzel_->setName("ouzel");
    ouzel_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
//    world_->addGround(); // we don't need a ground for the drone

    /// get robot data
    gcDim_ = ouzel_->getGeneralizedCoordinateDim();
    gvDim_ = ouzel_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    sampling_time_ = cfg["simulation_dt"].template As<float>();

    /// this is nominal configuration of anymal
    //TODO adapt nominal configuration of ouzel
    Eigen::Quaterniond init_quaternion = Eigen::Quaterniond::UnitRandom();
    gc_init_ << unifDistZeroOne_(gen_), unifDistZeroOne_(gen_), unifDistZeroOne_(gen_), // position
                init_quaternion.w(), init_quaternion.x(), init_quaternion.y(), init_quaternion.z(), //orientation quaternion
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    /// set reference_
    ref_position_ << unifDistZeroOne_(gen_), unifDistZeroOne_(gen_), unifDistZeroOne_(gen_);
    Eigen::Quaterniond ref_quaternion = Eigen::Quaterniond::UnitRandom();
    ref_orientation_ = ref_quaternion.toRotationMatrix();
    controller_.setRef(ref_position_, ref_quaternion);

    /// set pd gains
//    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
//    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);
//    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2);
//    anymal_->setPdGains(jointPgain, jointDgain);
//    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 30;
    actionDim_ =  9;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(actionDim_);
    actionStd_.setConstant(0.3);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground -> no ground
//    footIndices_.insert(anymal_->getBodyIdx("LF_SHANK"));
//    footIndices_.insert(anymal_->getBodyIdx("RF_SHANK"));
//    footIndices_.insert(anymal_->getBodyIdx("LH_SHANK"));
//    footIndices_.insert(anymal_->getBodyIdx("RH_SHANK"));

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(ouzel_);
    }
  }

  void init() final { }

  void reset() final {
    ouzel_->setState(gc_init_, gv_init_);
    updateObservation();
//    auto base_link_idx = ouzel_->getBodyIdx("ouzel/base_link");
//    raisim::Vec<3> base_link_pos_W;
//    ouzel_->getFramePosition(base_link_idx, base_link_pos_W);
//    std::cout << "none" << std::endl;
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    // give adapted waypoint to controller -> get wrench
//    std::cout << "action: " << action << std::endl;
    auto actionD = action.cast<double>();
    controller_.setOdom(obDouble_.segment(0, 3), obDouble_.segment(3, 9), obDouble_.segment(12, 3), obDouble_.segment(15, 3));
    controller_.setRefFromAction(actionD.segment(0, 3), actionD.segment(3, 3), actionD.segment(6, 3));
    mav_msgs::EigenTorqueThrust wrench_command;
    controller_.calculateWrenchCommand(&wrench_command, sampling_time_);
//    std::cout << "commanded thrust: " << wrench_command.thrust << " commanded torque: " << wrench_command.torque << std::endl;

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      // apply wrench on ouzel
      // TODO: check body index and wrench command frame
      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"), wrench_command.thrust);
      ouzel_->setExternalTorqueInBodyFrame(ouzel_->getBodyIdx("ouzel/base_link"), wrench_command.torque);

      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    // get observations
    updateObservation();

    Vec<3> position; Mat<3, 3> orientation;
    ouzel_->getLink("ouzel/base_link").getPosition(position);
//    std::cout << "waypointDist: " << (position.e() - ref_position_).squaredNorm() << std::endl;
    double waypoint_dist_clamped = std::min(1000.0, (position.e() - ref_position_).squaredNorm());
    rewards_.record("waypointDist", waypoint_dist_clamped);
//    rewards_.record("orientError", anymal_->g`etGeneralizedForce().squaredNorm());
//    rewards_.record("angularVel", anymal_->getGeneralizedForce().squaredNorm());
//    rewards_.record("force", anymal_->getGeneralizedForce().squaredNorm());
//    rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());

    return rewards_.sum();
  }

  void updateObservation() {
    // TODO add observations from sensor
    ouzel_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    obDouble_ << gc_.segment(0,3), /// body position
        rot.e().col(0), rot.e().col(1), rot.e().col(2), /// body orientation
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        ref_position_,/// ref position
        ref_orientation_.col(0), ref_orientation_.col(1), ref_orientation_.col(2); /// ref orientation

//    std::cout << "observation:\n" << obDouble_ << std::endl;
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
//    Eigen::VectorXf obFloat = obDouble_.cast<float>();
//    Eigen::Quaternionf quat(obFloat.segment(3, 4));
//    Eigen::Matrix3f rot_mat = quat.toRotationMatrix();
//    ob << obFloat.segment(0, 3), rot_mat.col(0), rot_mat.col(1), rot_mat.col(2), obFloat.segment(7, 6);
      ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    Vec<3> position;
    ouzel_->getLink("ouzel/base_link").getPosition(position);
    if((position.e() - ref_position_).squaredNorm() > terminalWaypointDist)
      return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* ouzel_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  double terminalWaypointDist = 5.0;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
  double sampling_time_;
  Eigen::Vector3d ref_position_;
  Eigen::Matrix3d ref_orientation_;

  rw_omav_controllers::ImpedanceControlModule controller_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  //  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
  std::uniform_real_distribution<double> unifDistZeroOne_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

