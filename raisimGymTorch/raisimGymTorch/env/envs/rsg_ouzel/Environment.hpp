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
#include "sensors.h"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), unifDistPlusMinusOne_(-1.0, 1.0), controller_(cfg["controller"]) {

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
    world_->addGround(-10.0); // we don't need a ground for the drone

    /// get robot data
    gcDim_ = ouzel_->getGeneralizedCoordinateDim();
    gvDim_ = ouzel_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
//    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    sampling_time_ = cfg["control_dt"].template As<float>();

    resetInitialConditions();

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
    resetInitialConditions();
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
    controller_.setOdom(obDouble_.segment(0, 3), obDouble_.segment(3, 9), obDouble_.segment(12, 3), obDouble_.segment(15, 3));

    auto actionD = action.cast<double>();
    Eigen::Vector3d ref_position_corr_vec = actionD.segment(0, 3);
    Eigen::Matrix3d ref_orientation_corr_mat = RotationMatrixFromTwoVectors(actionD.segment(3, 3), actionD.segment(6, 3));
    controller_.setRefFromAction(ref_position_corr_vec, ref_orientation_corr_mat);

    mav_msgs::EigenTorqueThrust wrench_command;
    controller_.calculateWrenchCommand(&wrench_command, sampling_time_);
//    std::cout << "commanded thrust:\n" << wrench_command.thrust << "commanded torque:\n" << wrench_command.torque << std::endl;

    Vec<3> orig;
    orig.setZero();
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      // apply wrench on ouzel
      // TODO: check body index and wrench command frame
      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"), ouzel_->BODY_FRAME, wrench_command.thrust, ouzel_->BODY_FRAME, orig); // set force in body frame
      ouzel_->setExternalTorqueInBodyFrame(ouzel_->getBodyIdx("ouzel/base_link"), wrench_command.torque);

      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    // get observations
    updateObservation();

    double waypoint_dist, error_angle;
    computeErrorMetrics(waypoint_dist, error_angle);
    Eigen::AngleAxisd ref_orientation_corr_angle_axis(ref_orientation_corr_mat);
    rewards_.record("waypointDist", waypoint_dist);
    rewards_.record("orientError", error_angle);
    rewards_.record("linearRefCorr", ref_position_corr_vec.squaredNorm());
    rewards_.record("orientRefCorr", ref_orientation_corr_angle_axis.angle());
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
        ref_position_, /// ref position
        ref_orientation_.col(0), ref_orientation_.col(1), ref_orientation_.col(2); /// ref orientation
//    std::cout << "ob quat:\n" << quat << std::endl;
//    std::cout << "observation:\n" << obDouble_ << std::endl;
//    std::cout << "lin vel:\n" << bodyLinearVel_ << std::endl;
//    std::cout << "ang vel:\n" << bodyAngularVel_ << std::endl;
    if (!Eigen::isfinite(gc_.array()).any()) {
      std::cout << "ob is nan!!" << std::endl;
      std::cout << "ob : " << obDouble_ << std::endl;
      std::cout << "gc : " << gc_ << std::endl;
      std::cout << "gv : " << gv_ << std::endl;
      raise(SIGTERM);
    }
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

    double waypoint_dist, error_angle;
    computeErrorMetrics(waypoint_dist, error_angle);
    if( waypoint_dist > terminalWaypointDist || error_angle > terminalAngleError)
      return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

  void setSeed(int seed) {
    gen_.seed(seed);
  }

 private:
  void resetInitialConditions() {
    Eigen::Vector3d init_position(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));

    Eigen::Vector3d init_orientation(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    init_orientation.normalize();
    double init_angle = unifDistPlusMinusOne_(gen_) * 2.0 * M_PI;
    Eigen::Quaterniond init_quaternion(Eigen::AngleAxisd(init_angle, init_orientation));
    init_quaternion.normalize();
    gc_init_ << init_position.x(), init_position.y(), init_position.z(), // position
            init_quaternion.w(), init_quaternion.x(), init_quaternion.y(), init_quaternion.z(), //orientation quaternion
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // reset reference
    Eigen::Vector3d ref_delta_position(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    ref_delta_position.normalize();
    ref_position_ = init_position + 1.5 * unifDistPlusMinusOne_(gen_) * ref_delta_position;

    Eigen::Vector3d ref_delta_orientation(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    ref_delta_orientation.normalize();
    double ref_delta_angle = unifDistPlusMinusOne_(gen_) * 45.0 / 180.0 * M_PI;
//    std::cout << "ref delta angle " << ref_delta_angle << std::endl;
//    std::cout << "ref delta pos\n" << ref_position_ - gc_init_.segment(0,3) << std::endl;
    Eigen::Quaterniond ref_delta_quaternion(Eigen::AngleAxisd(ref_delta_angle, ref_delta_orientation));
    ref_delta_quaternion.normalize();
    Eigen::Quaterniond ref_quaternion = init_quaternion * ref_delta_quaternion;
    ref_quaternion.normalize();

    ref_orientation_ = ref_quaternion.toRotationMatrix();
    controller_.setRef(ref_position_, ref_quaternion);
  }

  void computeErrorMetrics(double& waypointDist, double& errorAngle) {
    // Maybe want to clamp the error terms?
    waypointDist = (gc_.segment(0, 3) - ref_position_).squaredNorm();

    Eigen::Quaterniond current_quat(gc_[3], gc_[4], gc_[5], gc_[6]);
    auto error_quat = Eigen::Quaterniond (ref_orientation_) * current_quat.inverse();
    Eigen::AngleAxisd error_angle_axis(error_quat);
    errorAngle = error_angle_axis.angle();
  }

  Eigen::Matrix3d RotationMatrixFromTwoVectors(Eigen::Vector3d _orientation_vec_1, Eigen::Vector3d _orientation_vec_2) {
    Eigen::Matrix3d orientation_mat;
    orientation_mat.setIdentity();
    if (_orientation_vec_1.norm() > 0 && _orientation_vec_2.norm() > 0 && _orientation_vec_1.cross(_orientation_vec_2).norm() > 0) {
      // Gram-Schmidt
      Eigen::Vector3d e1 = _orientation_vec_1 / _orientation_vec_1.norm();
      Eigen::Vector3d u2 = _orientation_vec_2 - e1.dot(_orientation_vec_2) * e1;
      Eigen::Vector3d e2 = u2 / u2.norm();
      orientation_mat.col(0) = e1;
      orientation_mat.col(1) = e2;
      orientation_mat.col(2) = e1.cross(e2);
    }
    return orientation_mat;
  }


  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* ouzel_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  double terminalWaypointDist = 5.0;
  double terminalAngleError = 60.0 / 180.0 * M_PI;
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
  std::uniform_real_distribution<double> unifDistPlusMinusOne_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

