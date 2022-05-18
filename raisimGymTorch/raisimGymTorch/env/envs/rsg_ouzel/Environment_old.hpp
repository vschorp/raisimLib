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
#include "include/impedance_control_module.h"
#include "include/sensors.h"

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
//    std::cout << "gcDim_: " << gcDim_ << std::endl;
//    std::cout << "gvDim_: " << gvDim_ << std::endl;
//    std::cout << "nJoints_: " << nJoints_ << std::endl;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
//    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    control_dt_ = cfg["control_dt"].template As<float>();
    simulation_dt_ = cfg["simulation_dt"].template As<float>();

    /// Initialisation Parameters
    initialDistanceOffset_ = cfg["initialisation"]["distanceOffset"].template As<float>();
    initialAngularOffset_ = cfg["initialisation"]["angleOffsetDeg"].template As<float>() / 180.0 * M_PI;
    initialLinearVel_ = cfg["initialisation"]["linearVel"].template As<float>();
    initialAngularVel_ = cfg["initialisation"]["angularVelDeg"].template As<float>() / 180.0 * M_PI;

    resetInitialConditions();

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 27;
    actionDim_ =  9;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    position_W_.setZero();
    orientation_W_B_.setIdentity();
    bodyLinearVel_.setZero();
    bodyAngularVel_.setZero();

    /// action scaling
    actionMean_ = gc_init_.tail(actionDim_);
    actionStd_.setConstant(0.3);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);
    terminalOOBRewardCoeff_ = cfg["termination"]["oob"]["reward"].template As<float>();
    terminalOOBWaypointDist_ = cfg["termination"]["oob"]["waypointDist"].template As<float>();
    terminalOOBAngleError_ = cfg["termination"]["oob"]["angleErrorDeg"].template As<float>() / 180.0 * M_PI;
    terminalSuccessRewardCoeff_ = cfg["termination"]["success"]["reward"].template As<float>();
    terminalSuccessWaypointDist_ = cfg["termination"]["success"]["waypointDist"].template As<float>();
    terminalSuccessAngleError_ = cfg["termination"]["success"]["angleErrorDeg"].template As<float>() / 180.0 * M_PI;
    terminalSuccessLinearVel_ = cfg["termination"]["success"]["linearVel"].template As<float>();
    terminalSuccessAngularVel_ = cfg["termination"]["success"]["angularVelDeg"].template As<float>() / 180.0 * M_PI;

    // Add sensors
    auto* odometry_noise = new raisim_sensors::odometryNoise(control_dt_, cfg["odometryNoise"]);
    odometry_ = raisim_sensors::odometry(ouzel_, control_dt_, "ouzel", "ouzel/base_link", odometry_noise);

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
//    auto mass_vector = ouzel_->getMass();
//    auto com_W_vector = ouzel_->getBodyCOM_W();
//    auto inertia_B_vector = ouzel_->getInertia();
//    double total_mass = 0;
//    raisim::Vec<3> com_W = Eigen::Vector3d::Zero();
//    raisim::Vec<3> inertia_W = Eigen::Vector3d::Zero();
//    raisim::Vec<3> pos_W;
//    raisim::Vec<3> pos_body_W;
//    raisim::Vec<3> pos_body_W_squared;
//    for (int i = 0; i < mass_vector.size(); i++) {
//      total_mass += mass_vector[i];
//      com_W += com_W_vector[i] * mass_vector[i];
//      ouzel_->getPosition(i, pos_body_W);
//      pos_body_W -= ouzel_->getCOM();
//    }
//    com_W = com_W / total_mass;
//    ouzel_->getPosition(0, pos_W);
//    raisim::Vec<3> com_offset = com_W - pos_W;
//    auto a = ouzel_->getCompositeCOM();
//    auto b = ouzel_->getInertia();
//    std::cout << "ouzel mass: " << ouzel_->getCompositeMass()[0] << std::endl;
//    std::cout << "ouzel com W: " << ouzel_->getCOM() << std::endl;
//    std::cout << "ouzel com_offset: " <<com_offset<< std::endl;
//    std::cout << "ouzel inertia: " << ouzel_->getMassMatrix() << std::endl;
//    std::cout << "ouzel inertia: " << ouzel_->getMassMatrix().size() << std::endl;
//
//    int baseLink_ = ouzel_->getBodyIdx("ouzel/base_link");
//    raisim::Vec<3> com_pos = ouzel_->getCOM();
//    const std::vector<std::string> bodyList = ouzel_->getBodyNames();
//    raisim::Vec<3> base_link_pos_W;
//    ouzel_->getFramePosition(baseLink_, base_link_pos_W);
//    double Ixx=0, Iyy=0, Izz=0;
//    for (std::string bodyName : bodyList) {
//      size_t bodyIdx;
//      bodyIdx = ouzel_->getBodyIdx(bodyName);
//      raisim::Vec<3> point_W;
//      ouzel_->getFramePosition(bodyIdx, point_W);
//      double dx = point_W(0) - com_pos(0);
//      double dy = point_W(1) - com_pos(1);
//      double dz = point_W(2) - com_pos(2);
//      Ixx += (dy*dy + dz*dz)*ouzel_->getMass(bodyIdx);
//      Iyy += (dx*dx + dz*dz)*ouzel_->getMass(bodyIdx);
//      Izz += (dy*dy + dx*dx)*ouzel_->getMass(bodyIdx);
//    }
//
//    std::cout << "Inertia diagonal w.r.t. COM: "<<Ixx<<" "<<Iyy<<" "<<Izz << std::endl;
//    std::cout << "ouzel inertia: " << ouzel_->getCompositeInertia().back()<< std::endl;
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
//    controller_.setOdom(position_W_gt_, orientation_W_B_gt_, bodyLinearVel_gt_, bodyAngularVel_gt_);
    controller_.setOdom(position_W_, orientation_W_B_, bodyLinearVel_, bodyAngularVel_);

//    std::cout << "ref pos: " << ref_position_ << std::endl;
//    std::cout << "ref orient coeffs: " << ref_orientation_.coeffs() << std::endl;
//    double waypoint_distw, error_anglew;
//    computeErrorMetrics(waypoint_distw, error_anglew);
//    std::cout << "waypoint dist before action: " << waypoint_distw << std::endl;
//    std::cout << "angle error deg before action: " << error_anglew / M_PI * 180 << std::endl;

    auto actionD = action.cast<double>();
    Eigen::Vector3d ref_position_corr_vec = actionD.segment(0, 3);
    Eigen::Quaterniond ref_orientation_corr = QuaternionFromTwoVectors(actionD.segment(3, 3), actionD.segment(6, 3));
//    std::cout << "action ref pos: " << ref_position_corr_vec << std::endl;
//    std::cout << "action ref orient coeffs: " << ref_orientation_corr.coeffs() << std::endl;
    controller_.setRefFromAction(ref_position_corr_vec, ref_orientation_corr);

    mav_msgs::EigenTorqueThrust wrench_command;
    controller_.calculateWrenchCommand(&wrench_command, control_dt_);
//    std::cout << "commanded thrust:\n" << wrench_command.thrust << "\n" << "commanded torque:\n" << wrench_command.torque << std::endl;

//    std::cout << "base link idx: " << ouzel_->getBodyIdx("ouzel/base_link") << std::endl;
    Vec<3> orig;
    orig.setZero();
//    Eigen::Vector3d levitation_force_W(0, 0, ouzel_->getTotalMass() * 9.81);
//    Eigen::Vector3d levitation_force_B = orientation_W_B_gt_.inverse().toRotationMatrix() * levitation_force_W;
//    Eigen::Vector3d test_torque_W(0, 0, 0.1);
//    Eigen::Vector3d test_torque_B = orientation_W_B_gt_.inverse().toRotationMatrix() * test_torque_W;
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      // apply wrench on ouzel
      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"), ouzel_->BODY_FRAME, wrench_command.thrust, ouzel_->WORLD_FRAME, ouzel_->getCOM()); // set force in body frame
      ouzel_->setExternalTorqueInBodyFrame(ouzel_->getBodyIdx("ouzel/base_link"), wrench_command.torque);
//      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"), ouzel_->BODY_FRAME, wrench_command.thrust, ouzel_->BODY_FRAME, orig); // set force in body frame
//      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"), ouzel_->WORLD_FRAME, levitation_force_W, ouzel_->WORLD_FRAME, ouzel_->getCOM()); // set force in body frame
//      ouzel_->setExternalForce(ouzel_->getBodyIdx("ouzel/base_link"), ouzel_->BODY_FRAME, levitation_force_B, ouzel_->WORLD_FRAME, ouzel_->getCOM()); // set force in body frame
//      ouzel_->setExternalTorqueInBodyFrame(ouzel_->getBodyIdx("ouzel/base_link"), test_torque_B);
//      ouzel_->setExternalTorqueInBodyFrame(ouzel_->getBodyIdx("ouzel/base_link"), wrench_command.torque);

      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    // get observations
    updateObservation();

    double waypoint_dist, error_angle;
    computeErrorMetrics(waypoint_dist, error_angle);
    Eigen::AngleAxisd ref_orientation_corr_angle_axis(ref_orientation_corr);
//    std::cout << "waypoint dist: " << waypoint_dist << std::endl;
//    std::cout << "angle error deg: " << error_angle / M_PI * 180 << std::endl;
//    std::cout << "angle error: " << error_angle << std::endl;
//    std::cout << "ref orient corr angle: " << float(ref_orientation_corr_angle_axis.angle()) << std::endl;
    rewards_.record("waypointDist", float(waypoint_dist));
    rewards_.record("orientError", float(error_angle));
    rewards_.record("linearRefCorr", float(ref_position_corr_vec.squaredNorm()));
    rewards_.record("orientRefCorr", float(ref_orientation_corr_angle_axis.angle()));
//    rewards_.record("angularVel", anymal_->getGeneralizedForce().squaredNorm());
//    rewards_.record("force", anymal_->getGeneralizedForce().squaredNorm());
//    rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());

    return rewards_.sum();
  }

  void updateObservation() {
    odometry_.update();
    Eigen::VectorXd odometry_measurement = odometry_.getMeas();
    position_W_ = odometry_measurement.segment(0, 3);
    orientation_W_B_ = Eigen::Quaterniond(odometry_measurement(3), odometry_measurement(4), odometry_measurement(5), odometry_measurement(6)).normalized();
    bodyLinearVel_ = odometry_measurement.segment(7, 3);
    bodyAngularVel_ = odometry_measurement.segment(10, 3);

    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    position_W_gt_ = odometry_measurement_gt.segment(0, 3);
    orientation_W_B_gt_ = Eigen::Quaterniond(odometry_measurement_gt(3), odometry_measurement_gt(4), odometry_measurement_gt(5), odometry_measurement_gt(6)).normalized();
    bodyLinearVel_gt_ = odometry_measurement_gt.segment(7, 3);
    bodyAngularVel_gt_ = odometry_measurement_gt.segment(10, 3);

    ouzel_->getState(gc_, gv_);

//    std::cout << "odometry_measurement: " << odometry_measurement << std::endl;
//    std::cout << "position W: " << position_W_ << std::endl;
//    std::cout << "orientation_W_B_ coeffs: " << orientation_W_B_.coeffs() << std::endl;
//    std::cout << "bodyLinearVel_: " << bodyLinearVel_ << std::endl;
//    std::cout << "bodyAngularVel_: " << bodyAngularVel_ << std::endl;

//    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
//    std::cout << "odometry_measurement gt: " << odometry_measurement_gt << std::endl;

    if (!Eigen::isfinite(gc_.array()).all()) {
      std::cout << "ob is nan!!" << std::endl;
      std::cout << "odometry : " << odometry_measurement << std::endl;
      std::cout << "gc : " << gc_ << std::endl;
      std::cout << "gv : " << gv_ << std::endl;
      raise(SIGTERM);
    }
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    Eigen::Vector3d position_RC_W = position_W_ - ref_position_; // RC for ref to current
    Eigen::Matrix3d orientation_W_B_mat = orientation_W_B_.toRotationMatrix();
    Eigen::Matrix3d ref_orientation_mat = ref_orientation_.toRotationMatrix();
    Eigen::VectorXd ob_double(obDim_);
    ob_double << position_RC_W, orientation_W_B_mat.col(0), orientation_W_B_mat.col(1), orientation_W_B_mat.col(2),
                 bodyLinearVel_, bodyAngularVel_,
                 ref_orientation_mat.col(0), ref_orientation_mat.col(1), ref_orientation_mat.col(2);
    ob = ob_double.cast<float>();
//    std::cout << "orientation_W_B_ coeffs: \n" << orientation_W_B_.coeffs() << std::endl;
//    std::cout << "orientation_W_B_mat: \n" << orientation_W_B_mat << std::endl;
//    std::cout << "orientation_W_B_mat: " << orientation_W_B_mat << std::endl;
//    std::cout << "ob: " << ob << std::endl;
  }

  bool isTerminalState(float& terminalReward) final {
    double waypoint_dist, error_angle;
    computeErrorMetrics(waypoint_dist, error_angle);
    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    Eigen::Vector3d lin_vel_gt = odometry_measurement_gt.segment(7, 3);
    Eigen::Vector3d ang_vel_gt = odometry_measurement_gt.segment(10, 3);

    if(waypoint_dist > terminalOOBWaypointDist_ || error_angle > terminalOOBAngleError_) {
      terminalReward = terminalOOBRewardCoeff_;
//      std::cout << "termination oob" << std::endl;
      return true;
    }
    else if (waypoint_dist < terminalSuccessWaypointDist_ && error_angle < terminalSuccessAngleError_
             && lin_vel_gt.norm() < terminalSuccessLinearVel_ && ang_vel_gt.norm() < terminalSuccessAngularVel_) {
      terminalReward = terminalSuccessRewardCoeff_;
//      std::cout << "termination success" << std::endl;
      return true;
    }
    else {
      terminalReward = 0.f;
      return false;
    }
  }

  void curriculumUpdate() { };

  void setSeed(int seed) {
    gen_.seed(seed);
  }

 private:
  void resetInitialConditions() {
    Eigen::Vector3d init_position(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    position_W_ = init_position;

    Eigen::Vector3d init_orientation(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    init_orientation.normalize();
    double init_angle = unifDistPlusMinusOne_(gen_) * M_PI;
    Eigen::Quaterniond init_quaternion(Eigen::AngleAxisd(init_angle, init_orientation));
    init_quaternion.normalize();
//    init_quaternion = Eigen::Quaterniond::Identity();
    orientation_W_B_ = init_quaternion;

    Eigen::Vector3d init_lin_vel_dir(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_lin_vel = init_lin_vel_dir.normalized() * initialLinearVel_ * unifDistPlusMinusOne_(gen_);
    bodyLinearVel_ = init_lin_vel;
    Eigen::Vector3d init_lin_vel_W = orientation_W_B_.toRotationMatrix() * init_lin_vel;
//    std::cout << "init_lin_vel_W: " << init_lin_vel_W << std::endl;

    Eigen::Vector3d init_ang_vel_axis(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_ang_vel = init_ang_vel_axis.normalized() * initialAngularVel_ * unifDistPlusMinusOne_(gen_);
    bodyAngularVel_ = init_ang_vel;
    Eigen::Vector3d init_ang_vel_W = orientation_W_B_.toRotationMatrix() * init_ang_vel;
//    std::cout << "init_ang_vel_W: " << init_ang_vel_W << std::endl;

    gc_init_ << position_W_.x(), position_W_.y(), position_W_.z(), // position
            orientation_W_B_.w(), orientation_W_B_.x(), orientation_W_B_.y(), orientation_W_B_.z(), //orientation quaternion
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    gv_init_ << init_lin_vel_W.x(), init_lin_vel_W.y(), init_lin_vel_W.z(),
                init_ang_vel_W.x(), init_ang_vel_W.y(), init_ang_vel_W.z(),
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // reset reference
    Eigen::Vector3d ref_delta_position(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    ref_delta_position.normalize();
    ref_position_ = position_W_ + initialDistanceOffset_ * unifDistPlusMinusOne_(gen_) * ref_delta_position;

    Eigen::Vector3d ref_delta_orientation(unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_), unifDistPlusMinusOne_(gen_));
    ref_delta_orientation.normalize();
    double ref_delta_angle = unifDistPlusMinusOne_(gen_) * initialAngularOffset_;
//    std::cout << "ref delta angle " << ref_delta_angle << std::endl;
//    std::cout << "ref delta pos\n" << ref_position_ - gc_init_.segment(0,3) << std::endl;
    Eigen::Quaterniond ref_delta_quaternion(Eigen::AngleAxisd(ref_delta_angle, ref_delta_orientation));
    ref_delta_quaternion.normalize();
    Eigen::Quaterniond ref_quaternion = orientation_W_B_ * ref_delta_quaternion;
    ref_quaternion.normalize();
    ref_orientation_ = ref_quaternion;

    controller_.setRef(ref_position_, ref_orientation_);
//    std::cout << "ref_delta_angle deg: " << ref_delta_angle / M_PI * 180.0 << std::endl;
//    std::cout << "orientation_W_B_ coeffs: " << orientation_W_B_.coeffs() << std::endl;
//    std::cout << "init orientation_W_B_ coeffs: " << orientation_W_B_.coeffs() << std::endl;
//    std::cout << "ref_delta_quaternion coeffs: " << ref_delta_quaternion.coeffs() << std::endl;
//    std::cout << "ref_quaternion coeffs: " << ref_quaternion.coeffs() << std::endl;
//    double error_angle = ref_orientation_.angularDistance(orientation_W_B_);
//    std::cout << "initial error angle: " << error_angle<< std::endl;
  }

  void computeErrorMetrics(double& waypointDist, double& errorAngle) {
    // Maybe want to clamp the error terms?
    waypointDist = (position_W_gt_ - ref_position_).squaredNorm();

//    Eigen::Quaterniond current_quat(gc_[3], gc_[4], gc_[5], gc_[6]);
    errorAngle = orientation_W_B_gt_.angularDistance(ref_orientation_);
  }

  Eigen::Quaterniond QuaternionFromTwoVectors(Eigen::Vector3d _orientation_vec_1, Eigen::Vector3d _orientation_vec_2) {
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
    return Eigen::Quaterniond(orientation_mat);
  }


  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* ouzel_;
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
//  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
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

}

