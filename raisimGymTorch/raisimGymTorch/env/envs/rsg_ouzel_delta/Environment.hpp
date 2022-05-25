//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <functional>
#include <set>
#include <stdlib.h>

#include "RaisimGymEnv.hpp"
#include "include/delta_dynamics.h"
#include "include/impedance_control_module.h"
#include "include/sensors.h"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

public:
  explicit ENVIRONMENT(const std::string &resourceDir, const Yaml::Node &cfg,
                       bool visualizable)
      : RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable),
        unifDistPlusMinusOne_(-1.0, 1.0), controller_(cfg["controller"]) {
    render_ = cfg["render"].template As<bool>();

    /// create world
    world_ = std::make_unique<raisim::World>();
    /// add objects
    ouzel_ =
        world_->addArticulatedSystem(resourceDir_ + "/ouzel_delta/temp.urdf");

    ouzel_->setName("ouzel_delta");
    ouzel_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    world_->addGround(-10.0); // we don't need a ground for the drone
    baseLink_ = ouzel_->getBodyIdx("ouzel/base_link");

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

    /// initialize delta
    delta_sym_ =
        new delta_dynamics::DeltaController(cfg_["deltaArm"], control_dt_);
    ee_vel_prev_ = Eigen::Vector3d::Zero();
    B_om_WB_prev_ = Eigen::Vector3d::Zero();
    pos_offset_BD_ =
        Eigen::Vector3d(cfg["deltaArm"]["p_BO"]["x"].template As<float>(),
                        cfg["deltaArm"]["p_BO"]["y"].template As<float>(),
                        cfg["deltaArm"]["p_BO"]["z"].template As<float>());
    ang_offset_BD_ =
        Eigen::Quaterniond(cfg["deltaArm"]["q_BO"]["w"].template As<float>(),
                           cfg["deltaArm"]["q_BO"]["x"].template As<float>(),
                           cfg["deltaArm"]["q_BO"]["y"].template As<float>(),
                           cfg["deltaArm"]["q_BO"]["z"].template As<float>())
            .normalized();

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
    /// Sinus trajectory params
    min_sinus_amplitude_ = cfg["sinus_min_amplitude"].template As<float>();
    max_sinus_amplitude_ = cfg["sinus_max_amplitude"].template As<float>();
    min_lateral_speed_ = cfg["min_lateral_speed"].template As<float>();
    max_lateral_speed_ = cfg["max_lateral_speed"].template As<float>();
    min_sinus_wavelength_ = cfg["sinus_min_wavelength"].template As<float>();
    max_sinus_wavelength_ = cfg["sinus_max_wavelength"].template As<float>();
    ref_sampling_time_ = cfg["ref_sampling_time"].template As<float>();
    sinus_traj_share_ = cfg["sinus_traj_share"].template As<float>();

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 39;
    actionDim_ = 12;
    ouzel_position_W_.setZero();
    ouzel_orientation_W_B_.setIdentity();
    ouzel_linear_vel_B_.setZero();
    ouzel_angular_vel_B_.setZero();
    delta_ouzel_ref_position_offset_neutral_B << 0.0, 0.0, 0.4;
    delta_joint_angles_neutral_ << 0.7, 0.7, 0.7;
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
    min_time_in_success_state_ =
        cfg["termination"]["success"]["minTime"].template As<float>();

    // Add sensors
    auto *odometry_noise =
        new raisim_sensors::odometryNoise(control_dt_, cfg["odometryNoise"]);
    odometry_ = raisim_sensors::odometry(ouzel_, control_dt_, "ouzel",
                                         "ouzel/base_link", odometry_noise);

    delta_ouzel_ref_position_offset_previous_ = Eigen::Vector3d::Zero();
    ref_delta_joint_pos_previous_ = Eigen::Vector3d::Zero();
    time_in_success_state_ = 0.0;

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(ouzel_);
      if (render_) {
        delta_eef_ =
            server_->addVisualCylinder("delta_eef", 0.1, 0.008, 0.0, 0.0, 1.0);
        delta_eef_->setPosition(0, 0, 0);
        omav_ref_marker_ =
            server_->addVisualCylinder("omav_ref", 0.2, 0.008, 1.0, 0.0, 0.0);
        omav_ref_marker_->setPosition(0, 0, 0);
        delta_ee_ref_marker_ = server_->addVisualCylinder("delta_ee_ref", 0.15,
                                                          0.008, 0.0, 1.0, 0.0);
        delta_ee_ref_marker_->setPosition(0, 0, 0);
      }
    }

    is_sinus_traj_ = std::abs(unifDistPlusMinusOne_(gen_)) < sinus_traj_share_;
    resetInitialConditions();
  }

  void init() final {}

  void reset() final {
    is_sinus_traj_ = std::abs(unifDistPlusMinusOne_(gen_)) < sinus_traj_share_;
    resetInitialConditions();
    ouzel_->setState(gc_init_, gv_init_);
    time_in_success_state_ = 0.0;
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec> &action) final {
    controller_.setOdom(ouzel_position_W_, ouzel_orientation_W_B_,
                        ouzel_linear_vel_B_, ouzel_angular_vel_B_);

    auto actionD = action.cast<double>();
    Eigen::Vector3d delta_ouzel_ref_position_offset_W = actionD.segment(0, 3);
    Eigen::Quaterniond ref_ouzel_orientation_corr =
        QuaternionFromTwoVectors(actionD.segment(3, 3), actionD.segment(6, 3));
    controller_.adaptRefFromAction(delta_ouzel_ref_position_offset_W,
                                   ref_ouzel_orientation_corr);

    if (visualizable_ && render_) {
      Eigen::Vector3d pos =
          ref_delta_position_ + delta_ouzel_ref_position_offset_W;
      omav_ref_marker_->setPosition(pos(0), pos(1), pos(2));
      Eigen::Quaterniond eigen_quat =
          ref_ouzel_orientation_ * ref_ouzel_orientation_corr;
      eigen_quat.normalize();
      Eigen::Vector4d quat(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(),
                           eigen_quat.z());
      omav_ref_marker_->setOrientation(quat);
    }

    mav_msgs::EigenTorqueThrust wrench_command;
    controller_.calculateWrenchCommand(&wrench_command, control_dt_);

    // delta arm
    Eigen::Vector3d ref_delta_joint_pos_raw(actionD.tail(3));
    Eigen::Vector3d ref_delta_joint_pos =
        (Eigen::tanh(ref_delta_joint_pos_raw.array()) + 1.0) / 2.0 *
        delta_max_joint_angle_;
    //    Eigen::Vector3d ref_delta_joint_pos_clamped;
    //    ref_delta_joint_pos_clamped(0) = boost::algorithm::clamp(
    //        ref_delta_joint_pos(0), delta_min_joint_angle_,
    //        delta_max_joint_angle_);
    //    ref_delta_joint_pos_clamped(1) = boost::algorithm::clamp(
    //        ref_delta_joint_pos(1), delta_min_joint_angle_,
    //        delta_max_joint_angle_);
    //    ref_delta_joint_pos_clamped(2) = boost::algorithm::clamp(
    //        ref_delta_joint_pos(2), delta_min_joint_angle_,
    //        delta_max_joint_angle_);
    Eigen::VectorXd odometry_measurement_gt;
    Eigen::Vector3d force_B, torque_B;
    Eigen::Vector3d ee_pos, ee_vel, ee_acc, eef_pos_W;
    Eigen::Vector3d B_dv_WB, B_om_WB, B_dom_WB;
    for (int i = 0; i < int(control_dt_ / simulation_dt_ + 1e-10); i++) {
      odometry_.update();
      odometry_measurement_gt = odometry_.getMeasGT();
      ouzel_position_W_gt_ = odometry_measurement_gt.segment(0, 3);
      ouzel_orientation_W_B_gt_ = Eigen::Quaterniond(odometry_measurement_gt(3),
                                                     odometry_measurement_gt(4),
                                                     odometry_measurement_gt(5),
                                                     odometry_measurement_gt(6))
                                      .normalized();
      ouzel_linear_vel_B_gt_ = odometry_measurement_gt.segment(7, 3);
      ouzel_angular_vel_B_gt_ = odometry_measurement_gt.segment(10, 3);

      /// apply wrench on ouzel
      ouzel_->setExternalForce(baseLink_, ouzel_->BODY_FRAME,
                               wrench_command.thrust, ouzel_->WORLD_FRAME,
                               ouzel_->getCOM()); // set force in body frame
      ouzel_->setExternalTorqueInBodyFrame(baseLink_, wrench_command.torque);

      /// delta arm simulation
      delta_sym_->sendActuatorsCommand(ref_delta_joint_pos);

      delta_sym_->fwkinPosition(&ee_pos);
      delta_sym_->fwkinVel(&ee_vel, ee_pos);
      ee_acc = (ee_vel - ee_vel_prev_) / simulation_dt_;
      ee_vel_prev_ = ee_vel;

      B_dv_WB =
          ouzel_orientation_W_B_gt_.toRotationMatrix() * ouzel_linear_vel_B_gt_;
      B_om_WB = ouzel_orientation_W_B_gt_.toRotationMatrix() *
                ouzel_angular_vel_B_gt_;
      B_dom_WB = (B_om_WB - B_om_WB_prev_) / simulation_dt_;
      B_om_WB_prev_ = B_om_WB;

      /// Compute feed forward base wrench due to dynamics.
      delta_sym_->getBaseWrench(&force_B, &torque_B, ouzel_orientation_W_B_gt_,
                                B_om_WB, B_dom_WB, B_dv_WB, ee_pos, ee_vel,
                                ee_acc);
      ouzel_->setExternalForce(baseLink_, ouzel_->BODY_FRAME, force_B,
                               ouzel_->WORLD_FRAME, ouzel_->getCOM());
      ouzel_->setExternalTorqueInBodyFrame(baseLink_, torque_B);

      if (visualizable_ && render_) {
        eef_pos_W = ouzel_position_W_gt_ + pos_offset_BD_ +
                    ouzel_orientation_W_B_gt_.matrix() *
                        ang_offset_BD_.matrix() * ee_pos;
        delta_eef_->setPosition(eef_pos_W(0), eef_pos_W(1), eef_pos_W(2));
        Eigen::Vector4d quat(
            ouzel_orientation_W_B_gt_.w(), ouzel_orientation_W_B_gt_.x(),
            ouzel_orientation_W_B_gt_.y(), ouzel_orientation_W_B_gt_.z());
        delta_eef_->setOrientation(quat);
      }

      if (server_)
        server_->lockVisualizationServerMutex();
      world_->integrate();
      if (server_)
        server_->unlockVisualizationServerMutex();
    }

    if (is_sinus_traj_) {
      step_count_ += 1;
      if (step_count_ % int(ref_sampling_time_ / control_dt_ + 0.0001) == 0)
        updateReference();
    }
    updateObservation();

    Eigen::Vector3d delta_ouzel_ref_position_offset_diff;
    if (delta_ouzel_ref_position_offset_previous_ == Eigen::Vector3d::Zero()) {
      delta_ouzel_ref_position_offset_diff = Eigen::Vector3d::Zero();
    } else {
      delta_ouzel_ref_position_offset_diff =
          delta_ouzel_ref_position_offset_W -
          delta_ouzel_ref_position_offset_previous_;
    }
    delta_ouzel_ref_position_offset_previous_ =
        delta_ouzel_ref_position_offset_W;

    Eigen::Vector3d ref_delta_joint_pos_diff;
    if (ref_delta_joint_pos_previous_ == Eigen::Vector3d::Zero()) {
      ref_delta_joint_pos_diff = Eigen::Vector3d::Zero();
    } else {
      ref_delta_joint_pos_diff =
          ref_delta_joint_pos - ref_delta_joint_pos_previous_;
    }
    ref_delta_joint_pos_previous_ = ref_delta_joint_pos_diff;

    Eigen::Vector3d delta_ouzel_ref_position_offset_neutral_W =
        ouzel_orientation_W_B_gt_.normalized().toRotationMatrix() *
        delta_ouzel_ref_position_offset_neutral_B;

    /// compute rewards
    double waypoint_dist_delta, error_angle;
    computeErrorMetrics(waypoint_dist_delta, error_angle);
    Eigen::AngleAxisd ref_ouzel_orientation_corr_angle_axis(
        ref_ouzel_orientation_corr);
    rewards_.record("waypointDist", float(waypoint_dist_delta));
    rewards_.record("orientError", float(error_angle));
    rewards_.record("deltaOuzelRefPositionOffset",
                    float((delta_ouzel_ref_position_offset_W -
                           delta_ouzel_ref_position_offset_neutral_W)
                              .squaredNorm()));
    rewards_.record("deltaOuzelRefPositionOffsetDiff",
                    float(delta_ouzel_ref_position_offset_diff.squaredNorm()));
    rewards_.record("orientRefCorr",
                    float(ref_ouzel_orientation_corr_angle_axis.angle()));
    rewards_.record(
        "deltaJointAngles",
        float((ref_delta_joint_pos - delta_joint_angles_neutral_).norm()));
    rewards_.record("deltaJointAngularVel",
                    float(delta_joint_angular_vel_.norm()));
    return rewards_.sum();
  }

  void updateReference() {
    ref_delta_position_ = init_position_;
    double time = step_count_ * control_dt_;
    ref_delta_position_[linear_dir_] += lateral_speed_ * time;
    ref_delta_position_[oscillation_dir_] +=
        sinus_amplitude_ * std::sin(sinus_angular_freq_ * time + sinus_offset_);
    if (visualizable_ && render_) {
      delta_ee_ref_marker_->setPosition(ref_delta_position_(0),
                                        ref_delta_position_(1),
                                        ref_delta_position_(2));
    }
    controller_.setRef(ref_delta_position_, ref_ouzel_orientation_);
  }

  void updateObservation() {
    odometry_.update();
    Eigen::VectorXd odometry_measurement = odometry_.getMeas();
    ouzel_position_W_ = odometry_measurement.segment(0, 3);
    ouzel_orientation_W_B_ =
        Eigen::Quaterniond(odometry_measurement(3), odometry_measurement(4),
                           odometry_measurement(5), odometry_measurement(6))
            .normalized();
    ouzel_linear_vel_B_ = odometry_measurement.segment(7, 3);
    ouzel_angular_vel_B_ = odometry_measurement.segment(10, 3);

    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    ouzel_position_W_gt_ = odometry_measurement_gt.segment(0, 3);
    ouzel_orientation_W_B_gt_ = Eigen::Quaterniond(odometry_measurement_gt(3),
                                                   odometry_measurement_gt(4),
                                                   odometry_measurement_gt(5),
                                                   odometry_measurement_gt(6))
                                    .normalized();
    ouzel_linear_vel_B_gt_ = odometry_measurement_gt.segment(7, 3);
    ouzel_angular_vel_B_gt_ = odometry_measurement_gt.segment(10, 3);

    delta_joint_angle_ = delta_sym_->getqPos();
    delta_joint_angular_vel_ = delta_sym_->getqVel();

    Eigen::Vector3d end_effector_position_D;
    delta_sym_->fwkinPosition(&end_effector_position_D);

    delta_position_W_ = ouzel_position_W_ + pos_offset_BD_ +
                        ouzel_orientation_W_B_.matrix() *
                            ang_offset_BD_.matrix() * end_effector_position_D;
    delta_position_W_gt_ = ouzel_position_W_gt_ + pos_offset_BD_ +
                           ouzel_orientation_W_B_gt_.matrix() *
                               ang_offset_BD_.matrix() *
                               end_effector_position_D;

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
    Eigen::Vector3d ref_delta_position_offset_W =
        delta_position_W_ - ref_delta_position_; // ref to current
    Eigen::Vector3d delta_ouzel_position_offet_W =
        ouzel_position_W_ - delta_position_W_; // delta to ouzel
    Eigen::Matrix3d ouzel_orientation_W_B_mat =
        ouzel_orientation_W_B_.toRotationMatrix();
    Eigen::Matrix3d ref_ouzel_orientation_mat =
        ref_ouzel_orientation_.toRotationMatrix();
    Eigen::VectorXd ob_double(obDim_);
    ob_double << ref_delta_position_offset_W, delta_ouzel_position_offet_W,
        ouzel_orientation_W_B_mat.col(0), ouzel_orientation_W_B_mat.col(1),
        ouzel_orientation_W_B_mat.col(2), ouzel_linear_vel_B_,
        ouzel_angular_vel_B_, ref_ouzel_orientation_mat.col(0),
        ref_ouzel_orientation_mat.col(1), ref_ouzel_orientation_mat.col(2),
        delta_joint_angle_, delta_joint_angular_vel_, ref_delta_position_;
    ob = ob_double.cast<float>();
    if (!Eigen::isfinite(ob.array()).all()) {
      std::cout << "ob is nan: " << ob << std::endl;
    }
  }

  bool isTerminalState(float &terminalReward) final {
    double waypoint_dist_delta, error_angle;
    computeErrorMetrics(waypoint_dist_delta, error_angle);
    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    Eigen::Vector3d lin_vel_gt = odometry_measurement_gt.segment(7, 3);
    Eigen::Vector3d ang_vel_gt = odometry_measurement_gt.segment(10, 3);
    if (waypoint_dist_delta > terminalOOBWaypointDist_ ||
        error_angle > terminalOOBAngleError_) {
      terminalReward = terminalOOBRewardCoeff_;
      return true;
    } else if (waypoint_dist_delta < terminalSuccessWaypointDist_ &&
               error_angle < terminalSuccessAngleError_ &&
               lin_vel_gt.norm() < terminalSuccessLinearVel_ &&
               ang_vel_gt.norm() < terminalSuccessAngularVel_) {
      time_in_success_state_ += control_dt_;
      if (time_in_success_state_ >= min_time_in_success_state_) {
        terminalReward = terminalSuccessRewardCoeff_;
        return true;
      }
      terminalReward = 0.f;
      return false;
    } else {
      terminalReward = 0.f;
      time_in_success_state_ = 0.0;
      return false;
    }
  }

  void curriculumUpdate(){};

  void setSeed(int seed) { gen_.seed(seed); }

private:
  void resetInitialConditions() {
    init_position_ = Eigen::Vector3d(unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_));
    ouzel_position_W_ = init_position_;

    if (is_sinus_traj_) {
      ouzel_orientation_W_B_ = Eigen::Quaterniond::Identity();
    } else {
      Eigen::Vector3d init_orientation(unifDistPlusMinusOne_(gen_),
                                       unifDistPlusMinusOne_(gen_),
                                       unifDistPlusMinusOne_(gen_));
      init_orientation.normalize();
      double init_angle = unifDistPlusMinusOne_(gen_) * M_PI;
      Eigen::Quaterniond init_quaternion(
          Eigen::AngleAxisd(init_angle, init_orientation));
      init_quaternion.normalize();
      ouzel_orientation_W_B_ = init_quaternion;
    }

    Eigen::Vector3d init_lin_vel_dir(unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_lin_vel = init_lin_vel_dir.normalized() *
                                   initialLinearVel_ *
                                   unifDistPlusMinusOne_(gen_);
    ouzel_linear_vel_B_ = init_lin_vel;
    Eigen::Vector3d init_lin_vel_W =
        ouzel_orientation_W_B_.toRotationMatrix() * init_lin_vel;

    Eigen::Vector3d init_ang_vel_axis(unifDistPlusMinusOne_(gen_),
                                      unifDistPlusMinusOne_(gen_),
                                      unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_ang_vel = init_ang_vel_axis.normalized() *
                                   initialAngularVel_ *
                                   unifDistPlusMinusOne_(gen_);
    ouzel_angular_vel_B_ = init_ang_vel;
    Eigen::Vector3d init_ang_vel_W =
        ouzel_orientation_W_B_.toRotationMatrix() * init_ang_vel;

    gc_init_ << ouzel_position_W_.x(), ouzel_position_W_.y(),
        ouzel_position_W_.z(), // position
        ouzel_orientation_W_B_.w(), ouzel_orientation_W_B_.x(),
        ouzel_orientation_W_B_.y(),
        ouzel_orientation_W_B_.z(), // orientation quaternion
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    gv_init_ << init_lin_vel_W.x(), init_lin_vel_W.y(), init_lin_vel_W.z(),
        init_ang_vel_W.x(), init_ang_vel_W.y(), init_ang_vel_W.z(), 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // reset delta arm
    Eigen::Vector3d init_joint_angles(
        std::abs(unifDistPlusMinusOne_(gen_)) * 1.4,
        std::abs(unifDistPlusMinusOne_(gen_)) * 1.4,
        std::abs(unifDistPlusMinusOne_(gen_)) * 1.4);
    delta_sym_->setJointAngles(init_joint_angles);

    // reset reference
    if (is_sinus_traj_) {
      linear_dir_ = int(std::abs(unifDistPlusMinusOne_(gen_)) * 2.999);
      bool oscillation_dir_proxy = std::signbit(unifDistPlusMinusOne_(gen_));
      std::vector<int> all_oscillation_dir{2, 0, 1, 2, 0};
      oscillation_dir_ =
          all_oscillation_dir[linear_dir_ + 1 + int(oscillation_dir_proxy) -
                              int(!oscillation_dir_proxy)];

      sinus_amplitude_ = std::abs(unifDistPlusMinusOne_(gen_)) *
                             (max_sinus_amplitude_ - min_sinus_amplitude_) +
                         min_sinus_amplitude_;
      lateral_speed_ = std::abs(unifDistPlusMinusOne_(gen_)) *
                           (max_lateral_speed_ - min_lateral_speed_) +
                       min_lateral_speed_;
      double sinus_wavelength =
          std::abs(unifDistPlusMinusOne_(gen_)) *
              (max_sinus_wavelength_ - min_sinus_wavelength_) +
          min_sinus_wavelength_;
      sinus_offset_ = unifDistPlusMinusOne_(gen_) * M_PI;
      sinus_angular_freq_ = 2 * M_PI * lateral_speed_ / sinus_wavelength;
      step_count_ = 0;

      ref_delta_position_ = ouzel_position_W_;
      ref_ouzel_orientation_ = ouzel_orientation_W_B_;
    } else {
      Eigen::Vector3d ref_delta_position(unifDistPlusMinusOne_(gen_),
                                         unifDistPlusMinusOne_(gen_),
                                         unifDistPlusMinusOne_(gen_));
      ref_delta_position.normalize();
      ref_delta_position_ =
          ouzel_position_W_ + initialDistanceOffset_ *
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
      Eigen::Quaterniond ref_quaternion =
          ouzel_orientation_W_B_ * ref_delta_quaternion;
      ref_quaternion.normalize();
      ref_ouzel_orientation_ = ref_quaternion;
    }
    if (visualizable_ && render_) {
      delta_ee_ref_marker_->setPosition(ref_delta_position_(0),
                                        ref_delta_position_(1),
                                        ref_delta_position_(2));
      Eigen::Vector4d quat(
          ref_ouzel_orientation_.w(), ref_ouzel_orientation_.x(),
          ref_ouzel_orientation_.y(), ref_ouzel_orientation_.z());
      delta_ee_ref_marker_->setOrientation(quat);
    }
    controller_.setRef(ref_delta_position_, ref_ouzel_orientation_);
  }

  void computeErrorMetrics(double &waypointDist, double &errorAngle) {
    waypointDist = (delta_position_W_gt_ - ref_delta_position_).norm();
    errorAngle =
        ouzel_orientation_W_B_gt_.angularDistance(ref_ouzel_orientation_);
  }

  Eigen::Quaterniond
  QuaternionFromTwoVectors(Eigen::Vector3d _orientation_vec_1,
                           Eigen::Vector3d _orientation_vec_2) {
    Eigen::Matrix3d orientation_mat;
    orientation_mat.setIdentity();
    if (_orientation_vec_1.norm() > 0 && _orientation_vec_2.norm() > 0 &&
        _orientation_vec_1.cross(_orientation_vec_2).norm() > 0) {
      /// Gram-Schmidt
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
  bool render_ = false;
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
  Eigen::Vector3d init_position_;
  Eigen::Vector3d ouzel_position_W_, ouzel_linear_vel_B_, ouzel_angular_vel_B_,
      delta_joint_angle_, delta_joint_angular_vel_;
  Eigen::Quaterniond ouzel_orientation_W_B_;
  Eigen::Vector3d ouzel_position_W_gt_, ouzel_linear_vel_B_gt_,
      ouzel_angular_vel_B_gt_;
  Eigen::Quaterniond ouzel_orientation_W_B_gt_;
  std::set<size_t> footIndices_;
  Eigen::Vector3d ref_delta_position_;
  Eigen::Quaterniond ref_ouzel_orientation_;
  Eigen::Vector3d delta_ouzel_ref_position_offset_previous_;
  Eigen::Vector3d ref_delta_joint_pos_previous_;

  rw_omav_controllers::ImpedanceControlModule controller_;

  raisim_sensors::odometry odometry_;

  delta_dynamics::DeltaController *delta_sym_;
  raisim::Visuals *delta_eef_;
  raisim::Visuals *omav_ref_marker_;
  raisim::Visuals *delta_ee_ref_marker_;
  Eigen::Vector3d ee_vel_prev_;
  Eigen::Vector3d B_om_WB_prev_;
  Eigen::Vector3d pos_offset_BD_;
  Eigen::Quaterniond ang_offset_BD_;
  //  double delta_min_joint_angle_ = 0.0;
  double delta_max_joint_angle_ = 1.4; // rad
  Eigen::Vector3d delta_position_W_;
  Eigen::Vector3d delta_position_W_gt_;
  Eigen::Vector3d delta_joint_angles_neutral_;

  int baseLink_;

  Eigen::Vector3d delta_ouzel_ref_position_offset_neutral_B;

  float time_in_success_state_;
  float min_time_in_success_state_;

  int linear_dir_;
  int oscillation_dir_;

  double sinus_amplitude_;
  double lateral_speed_;
  double sinus_angular_freq_;
  double sinus_offset_;
  double min_sinus_amplitude_;
  double max_sinus_amplitude_;
  double min_lateral_speed_;
  double max_lateral_speed_;
  double min_sinus_wavelength_;
  double max_sinus_wavelength_;
  float ref_sampling_time_;
  int step_count_;
  double sinus_traj_share_;
  bool is_sinus_traj_;

  thread_local static std::mt19937 gen_;
  std::uniform_real_distribution<double> unifDistPlusMinusOne_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

} // namespace raisim
