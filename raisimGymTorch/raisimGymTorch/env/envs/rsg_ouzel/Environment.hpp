//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <algorithm>
#include <functional>
#include <set>
#include <stdlib.h>

#include "../../RaisimGymEnv.hpp"
#include "include/impedance_control_module.h"
#include "include/sensors.h"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

public:
  explicit ENVIRONMENT(const std::string &resourceDir, const Yaml::Node &cfg,
                       bool visualizable)
      : RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable),
        unifDistPlusMinusOne_(-1.0, 1.0), controller_(cfg["controller"]) {
    /// create world
    world_ = std::make_unique<raisim::World>();
    /// add objects
    ouzel_ = world_->addArticulatedSystem(resourceDir_ + "/ouzel/temp.urdf");

    ouzel_->setName("ouzel");
    ouzel_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    world_->addGround(-10.0); // we don't need a ground for the drone

    /// get robot data
    gcDim_ = ouzel_->getGeneralizedCoordinateDim();
    gvDim_ = ouzel_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_);
    gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_);
    gv_init_.setZero(gvDim_);
    control_dt_ = cfg["control_dt"].template As<float>();
    simulation_dt_ = cfg["simulation_dt"].template As<float>();

    /// Initialisation Parameters
    initialDistanceOffset_ =
        cfg["initialisation"]["distanceOffset"].template As<float>();
    initialAngularOffset_ =
        cfg["initialisation"]["angleOffsetDeg"].template As<float>() / 180.0 *
        M_PI;
    initialLinearVel_ = cfg["initialisation"]["linearVel"].template As<float>();
    initialAngularVel_ =
        cfg["initialisation"]["angularVelDeg"].template As<float>() / 180.0 *
        M_PI;

    resetInitialConditions();

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 27;
    actionDim_ = 9;
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    position_W_.setZero();
    orientation_W_B_.setIdentity();
    bodyLinearVel_.setZero();
    bodyAngularVel_.setZero();

    /// action scaling
    actionMean_ = gc_init_.tail(actionDim_);
    actionStd_.setConstant(0.3);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile(cfg["reward"]);
    terminalOOBRewardCoeff_ =
        cfg["termination"]["oob"]["reward"].template As<float>();
    terminalOOBWaypointDist_ =
        cfg["termination"]["oob"]["waypointDist"].template As<float>();
    terminalOOBAngleError_ =
        cfg["termination"]["oob"]["angleErrorDeg"].template As<float>() /
        180.0 * M_PI;
    terminalSuccessRewardCoeff_ =
        cfg["termination"]["success"]["reward"].template As<float>();
    terminalSuccessWaypointDist_ =
        cfg["termination"]["success"]["waypointDist"].template As<float>();
    terminalSuccessAngleError_ =
        cfg["termination"]["success"]["angleErrorDeg"].template As<float>() /
        180.0 * M_PI;
    terminalSuccessLinearVel_ =
        cfg["termination"]["success"]["linearVel"].template As<float>();
    terminalSuccessAngularVel_ =
        cfg["termination"]["success"]["angularVelDeg"].template As<float>() /
        180.0 * M_PI;

    // Add sensors
    auto *odometry_noise =
        new raisim_sensors::odometryNoise(control_dt_, cfg["odometryNoise"]);
    odometry_ = raisim_sensors::odometry(ouzel_, control_dt_, "ouzel",
                                         "ouzel/base_link", odometry_noise);

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(ouzel_);
    }
  }

  void init() final {}

  void reset() final {
    resetInitialConditions();
    ouzel_->setState(gc_init_, gv_init_);
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec> &action) final {
    controller_.setOdom(position_W_, orientation_W_B_, bodyLinearVel_,
                        bodyAngularVel_);
    auto actionD = action.cast<double>();
    Eigen::Vector3d ref_position_corr_vec = actionD.segment(0, 3);
    Eigen::Quaterniond ref_orientation_corr =
        QuaternionFromTwoVectors(actionD.segment(3, 3), actionD.segment(6, 3));
    controller_.setRefFromAction(ref_position_corr_vec, ref_orientation_corr);

    mav_msgs::EigenTorqueThrust wrench_command;
    controller_.calculateWrenchCommand(&wrench_command, control_dt_);

    for (int i = 0; i < int(control_dt_ / simulation_dt_ + 1e-10); i++) {
      // apply wrench on ouzel
      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"),
                               ouzel_->BODY_FRAME, wrench_command.thrust,
                               ouzel_->WORLD_FRAME,
                               ouzel_->getCOM()); // set force in body frame
      ouzel_->setExternalTorqueInBodyFrame(
          ouzel_->getBodyIdx("ouzel/base_link"), wrench_command.torque);

      if (server_)
        server_->lockVisualizationServerMutex();
      world_->integrate();
      if (server_)
        server_->unlockVisualizationServerMutex();
    }

    // get observations
    updateObservation();

    double waypoint_dist, error_angle;
    computeErrorMetrics(waypoint_dist, error_angle);
    Eigen::AngleAxisd ref_orientation_corr_angle_axis(ref_orientation_corr);
    rewards_.record("waypointDist", float(waypoint_dist));
    rewards_.record("orientError", float(error_angle));
    rewards_.record("linearRefCorr",
                    float(ref_position_corr_vec.squaredNorm()));
    rewards_.record("orientRefCorr",
                    float(ref_orientation_corr_angle_axis.angle()));
    return rewards_.sum();
  }

  void updateObservation() {
    odometry_.update();
    Eigen::VectorXd odometry_measurement = odometry_.getMeas();
    position_W_ = odometry_measurement.segment(0, 3);
    orientation_W_B_ =
        Eigen::Quaterniond(odometry_measurement(3), odometry_measurement(4),
                           odometry_measurement(5), odometry_measurement(6))
            .normalized();
    bodyLinearVel_ = odometry_measurement.segment(7, 3);
    bodyAngularVel_ = odometry_measurement.segment(10, 3);

    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    position_W_gt_ = odometry_measurement_gt.segment(0, 3);
    orientation_W_B_gt_ = Eigen::Quaterniond(odometry_measurement_gt(3),
                                             odometry_measurement_gt(4),
                                             odometry_measurement_gt(5),
                                             odometry_measurement_gt(6))
                              .normalized();
    bodyLinearVel_gt_ = odometry_measurement_gt.segment(7, 3);
    bodyAngularVel_gt_ = odometry_measurement_gt.segment(10, 3);

    ouzel_->getState(gc_, gv_);

    if (!Eigen::isfinite(gc_.array()).all()) {
      std::cout << "gc is nan!!" << std::endl;
      std::cout << "odometry : " << odometry_measurement << std::endl;
      std::cout << "gc : " << gc_ << std::endl;
      std::cout << "gv : " << gv_ << std::endl;
      raise(SIGTERM);
    }
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    Eigen::Vector3d position_RC_W =
        position_W_ - ref_position_; // RC for ref to current
    Eigen::Matrix3d orientation_W_B_mat = orientation_W_B_.toRotationMatrix();
    Eigen::Matrix3d ref_orientation_mat = ref_orientation_.toRotationMatrix();
    Eigen::VectorXd ob_double(obDim_);
    ob_double << position_RC_W, orientation_W_B_mat.col(0),
        orientation_W_B_mat.col(1), orientation_W_B_mat.col(2), bodyLinearVel_,
        bodyAngularVel_, ref_orientation_mat.col(0), ref_orientation_mat.col(1),
        ref_orientation_mat.col(2);
    ob = ob_double.cast<float>();
  }

  bool isTerminalState(float &terminalReward) final {
    double waypoint_dist, error_angle;
    computeErrorMetrics(waypoint_dist, error_angle);
    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    Eigen::Vector3d lin_vel_gt = odometry_measurement_gt.segment(7, 3);
    Eigen::Vector3d ang_vel_gt = odometry_measurement_gt.segment(10, 3);

    if (waypoint_dist > terminalOOBWaypointDist_ ||
        error_angle > terminalOOBAngleError_) {
      terminalReward = terminalOOBRewardCoeff_;
      return true;
    } else if (waypoint_dist < terminalSuccessWaypointDist_ &&
               error_angle < terminalSuccessAngleError_ &&
               lin_vel_gt.norm() < terminalSuccessLinearVel_ &&
               ang_vel_gt.norm() < terminalSuccessAngularVel_) {
      terminalReward = terminalSuccessRewardCoeff_;
      return true;
    } else {
      terminalReward = 0.f;
      return false;
    }
  }

  void curriculumUpdate(){};

  void setSeed(int seed) { gen_.seed(seed); }

private:
  void resetInitialConditions() {
    Eigen::Vector3d init_position(unifDistPlusMinusOne_(gen_),
                                  unifDistPlusMinusOne_(gen_),
                                  unifDistPlusMinusOne_(gen_));
    position_W_ = init_position;

    Eigen::Vector3d init_orientation(unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_));
    init_orientation.normalize();
    double init_angle = unifDistPlusMinusOne_(gen_) * M_PI;
    Eigen::Quaterniond init_quaternion(
        Eigen::AngleAxisd(init_angle, init_orientation));
    init_quaternion.normalize();
    orientation_W_B_ = init_quaternion;

    Eigen::Vector3d init_lin_vel_dir(unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_lin_vel = init_lin_vel_dir.normalized() *
                                   initialLinearVel_ *
                                   unifDistPlusMinusOne_(gen_);
    bodyLinearVel_ = init_lin_vel;
    Eigen::Vector3d init_lin_vel_W =
        orientation_W_B_.toRotationMatrix() * init_lin_vel;

    Eigen::Vector3d init_ang_vel_axis(unifDistPlusMinusOne_(gen_),
                                      unifDistPlusMinusOne_(gen_),
                                      unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_ang_vel = init_ang_vel_axis.normalized() *
                                   initialAngularVel_ *
                                   unifDistPlusMinusOne_(gen_);
    bodyAngularVel_ = init_ang_vel;
    Eigen::Vector3d init_ang_vel_W =
        orientation_W_B_.toRotationMatrix() * init_ang_vel;

    gc_init_ << position_W_.x(), position_W_.y(), position_W_.z(), // position
        orientation_W_B_.w(), orientation_W_B_.x(), orientation_W_B_.y(),
        orientation_W_B_.z(), // orientation quaternion
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    gv_init_ << init_lin_vel_W.x(), init_lin_vel_W.y(), init_lin_vel_W.z(),
        init_ang_vel_W.x(), init_ang_vel_W.y(), init_ang_vel_W.z(), 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // reset reference
    Eigen::Vector3d ref_delta_position(unifDistPlusMinusOne_(gen_),
                                       unifDistPlusMinusOne_(gen_),
                                       unifDistPlusMinusOne_(gen_));
    ref_delta_position.normalize();
    ref_position_ = position_W_ + initialDistanceOffset_ *
                                      unifDistPlusMinusOne_(gen_) *
                                      ref_delta_position;

    Eigen::Vector3d ref_delta_orientation(unifDistPlusMinusOne_(gen_),
                                          unifDistPlusMinusOne_(gen_),
                                          unifDistPlusMinusOne_(gen_));
    ref_delta_orientation.normalize();
    double ref_delta_angle =
        unifDistPlusMinusOne_(gen_) * initialAngularOffset_;

    Eigen::Quaterniond ref_delta_quaternion(
        Eigen::AngleAxisd(ref_delta_angle, ref_delta_orientation));
    ref_delta_quaternion.normalize();
    Eigen::Quaterniond ref_quaternion = orientation_W_B_ * ref_delta_quaternion;
    ref_quaternion.normalize();
    ref_orientation_ = ref_quaternion;

    controller_.setRef(ref_position_, ref_orientation_);
  }

  void computeErrorMetrics(double &waypointDist, double &errorAngle) {
    waypointDist = (position_W_gt_ - ref_position_).squaredNorm();
    errorAngle = orientation_W_B_gt_.angularDistance(ref_orientation_);
  }

  Eigen::Quaterniond
  QuaternionFromTwoVectors(Eigen::Vector3d _orientation_vec_1,
                           Eigen::Vector3d _orientation_vec_2) {
    Eigen::Matrix3d orientation_mat;
    orientation_mat.setIdentity();
    if (_orientation_vec_1.norm() > 0 && _orientation_vec_2.norm() > 0 &&
        _orientation_vec_1.cross(_orientation_vec_2).norm() > 0) {
      // Gram-Schmidt
      Eigen::Vector3d e1 = _orientation_vec_1 / _orientation_vec_1.norm();
      Eigen::Vector3d u2 = _orientation_vec_2 - e1.dot(_orientation_vec_2) * e1;
      Eigen::Vector3d e2 = u2 / u2.norm();
      orientation_mat.col(0) = e1;
      orientation_mat.col(1) = e2;
      orientation_mat.col(2) = e1.cross(e2);
    }
    return Eigen::Quaterniond(orientation_mat);
  }

  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem *ouzel_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double initialDistanceOffset_;
  double initialAngularOffset_;
  double initialLinearVel_;
  double initialAngularVel_;
  float terminalOOBRewardCoeff_;
  float terminalSuccessRewardCoeff_;
  double terminalOOBWaypointDist_;
  double terminalOOBAngleError_;
  double terminalSuccessWaypointDist_;
  double terminalSuccessAngleError_;
  double terminalSuccessLinearVel_;
  double terminalSuccessAngularVel_;
  Eigen::VectorXd actionMean_, actionStd_;
  Eigen::Vector3d position_W_, bodyLinearVel_, bodyAngularVel_;
  Eigen::Quaterniond orientation_W_B_;
  Eigen::Vector3d position_W_gt_, bodyLinearVel_gt_, bodyAngularVel_gt_;
  Eigen::Quaterniond orientation_W_B_gt_;
  std::set<size_t> footIndices_;
  Eigen::Vector3d ref_position_;
  Eigen::Quaterniond ref_orientation_;

  rw_omav_controllers::ImpedanceControlModule controller_;

  raisim_sensors::odometry odometry_;

  //  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
  std::uniform_real_distribution<double> unifDistPlusMinusOne_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

} // namespace raisim
