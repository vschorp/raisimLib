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
    //    parseURDF();
    //    std::ofstream myfile;
    //    myfile.open (ros::package::getPath("ros_raisim_interface") +
    //    "/resource/temp.urdf", std::ios::out); myfile << *urdf_;
    //    myfile.close();
    ouzel_ =
        world_->addArticulatedSystem(resourceDir_ + "/ouzel_delta/temp.urdf");

    //    std::string baseLink = ouzel_->getBodyIdx("ouzel/base_link");
    //    ouzel_ =
    //    world_->addArticulatedSystem(resourceDir_+"/ouzel/urdf/model.urdf");
    //    //used to be anymal.urdf
    ouzel_->setName("ouzel_delta_sinus");
    ouzel_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    world_->addGround(-10.0); // we don't need a ground for the drone
    baseLink_ = ouzel_->getBodyIdx("ouzel/base_link");

    /// get robot data
    gcDim_ = ouzel_->getGeneralizedCoordinateDim();
    gvDim_ = ouzel_->getDOF();
    nJoints_ = gvDim_ - 6;
    //    std::cout << "gcDim_: " << gcDim_ << std::endl;
    //    std::cout << "gvDim_: " << gvDim_ << std::endl;
    //    std::cout << "nJoints_: " << nJoints_ << std::endl;

    /// initialize containers
    gc_.setZero(gcDim_);
    gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_);
    gv_init_.setZero(gvDim_);
    max_sinus_amplitude_ = cfg["sinus_max_amplitude"].template As<float>();
    max_lateral_speed_ = cfg["max_lateral_speed"].template As<float>();
    max_sinus_wavelength_ = cfg["sinus_max_wavelength"].template As<float>();
    ref_sampling_time_ = cfg["ref_sampling_time"].template As<float>();

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
    //    std::cout << "pos_offset_BD_: " << pos_offset_BD_ << std::endl;
    ang_offset_BD_ =
        Eigen::Quaterniond(cfg["deltaArm"]["q_BO"]["w"].template As<float>(),
                           cfg["deltaArm"]["q_BO"]["x"].template As<float>(),
                           cfg["deltaArm"]["q_BO"]["y"].template As<float>(),
                           cfg["deltaArm"]["q_BO"]["z"].template As<float>())
            .normalized();
    //    std::cout << "ang_offset_BD_.matrix(): " << ang_offset_BD_.matrix()
    //              << std::endl;
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

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 36;
    actionDim_ = 12;
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    ouzel_position_W_.setZero();
    ouzel_orientation_W_B_.setIdentity();
    ouzel_linear_vel_B_.setZero();
    ouzel_angular_vel_B_.setZero();

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

    resetInitialConditions();

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
    //    std::cout << "ouzel mass: " << ouzel_->getCompositeMass()[0] <<
    //    std::endl; std::cout << "ouzel com W: " << ouzel_->getCOM() <<
    //    std::endl; std::cout << "ouzel com_offset: " << com_offset <<
    //    std::endl;
    //    //    std::cout << "ouzel inertia: " << ouzel_->getMassMatrix() <<
    //    //    std::endl; std::cout << "ouzel inertia: " <<
    //    //    ouzel_->getMassMatrix().size()
    //    //              << std::endl;
    //
    //    //    int baseLink_ = ouzel_->getBodyIdx("ouzel/base_link");
    //    raisim::Vec<3> com_pos = ouzel_->getCOM();
    //    const std::vector<std::string> bodyList = ouzel_->getBodyNames();
    //    raisim::Vec<3> base_link_pos_W;
    //    ouzel_->getFramePosition(baseLink_, base_link_pos_W);
    //    double Ixx = 0, Iyy = 0, Izz = 0;
    //    for (std::string bodyName : bodyList) {
    //      size_t bodyIdx;
    //      bodyIdx = ouzel_->getBodyIdx(bodyName);
    //      raisim::Vec<3> point_W;
    //      ouzel_->getFramePosition(bodyIdx, point_W);
    //      double dx = point_W(0) - com_pos(0);
    //      double dy = point_W(1) - com_pos(1);
    //      double dz = point_W(2) - com_pos(2);
    //      Ixx += (dy * dy + dz * dz) * ouzel_->getMass(bodyIdx);
    //      Iyy += (dx * dx + dz * dz) * ouzel_->getMass(bodyIdx);
    //      Izz += (dy * dy + dx * dx) * ouzel_->getMass(bodyIdx);
    //    }
    //
    //    std::cout << "Inertia diagonal w.r.t. COM: " << Ixx << " " << Iyy << "
    //    "
    //              << Izz << std::endl;
    //    std::cout << "ouzel inertia: " << ouzel_->getCompositeInertia()[0]
    //              << std::endl;
  }

  void init() final {}

  void reset() final {
    resetInitialConditions();
    ouzel_->setState(gc_init_, gv_init_);
    //    std::cout << "gc init: " << gc_init_ << std::endl;
    //    std::cout << "gv init: " << gv_init_ << std::endl;
    time_in_success_state_ = 0.0;
    updateObservation();
    //    std::cout << "end of reset" << std::endl;
    //    auto base_link_idx = ouzel_->getBodyIdx("ouzel/base_link");
    //    raisim::Vec<3> base_link_pos_W;
    //    ouzel_->getFramePosition(base_link_idx, base_link_pos_W);
    //    std::cout << "none" << std::endl;
  }

  float step(const Eigen::Ref<EigenVec> &action) final {
    // give adapted waypoint to controller -> get wrench
    //    std::cout << "action: " << action << std::endl;
    //    controller_.setOdom(ouzel_position_W_gt_, ouzel_orientation_W_B_gt_,
    //    ouzel_linear_vel_B_gt_, ouzel_angular_vel_B_gt_);
    controller_.setOdom(ouzel_position_W_, ouzel_orientation_W_B_,
                        ouzel_linear_vel_B_, ouzel_angular_vel_B_);

    //    std::cout << "ref pos: " << ref_delta_position_ << std::endl;
    //    std::cout << "ref orient coeffs: " << ref_ouzel_orientation_.coeffs()
    //    << std::endl; double waypoint_dist_deltaw, error_anglew;
    //    computeErrorMetrics(waypoint_dist_deltaw, error_anglew);
    //    std::cout << "waypoint dist before action: " << waypoint_dist_deltaw
    //    << std::endl; std::cout << "angle error deg before action: " <<
    //    error_anglew / M_PI * 180 << std::endl;

    auto actionD = action.cast<double>();
    //    Eigen::Vector3d delta_ouzel_ref_position_offset =
    //        Eigen::Vector3d(0.0, 0.0, 0.3);
    Eigen::Vector3d delta_ouzel_ref_position_offset = actionD.segment(0, 3);
    Eigen::Quaterniond ref_ouzel_orientation_corr =
        QuaternionFromTwoVectors(actionD.segment(3, 3), actionD.segment(6, 3));
    //    std::cout << "action ref pos: " << delta_ouzel_ref_position_offset <<
    //    std::endl; std::cout << "action ref orient coeffs: " <<
    //    ref_ouzel_orientation_corr.coeffs() << std::endl;
    controller_.adaptRefFromAction(delta_ouzel_ref_position_offset,
                                   ref_ouzel_orientation_corr);

    if (visualizable_ && render_) {
      Eigen::Vector3d pos =
          ref_delta_position_ + delta_ouzel_ref_position_offset;
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
    //    std::cout << "commanded thrust:\n"
    //              << wrench_command.thrust << "\n"
    //              << "commanded torque:\n"
    //              << wrench_command.torque << std::endl;

    // delta arm
    Eigen::Vector3d ref_delta_joint_pos(actionD.tail(3));
    Eigen::Vector3d ref_delta_joint_pos_clamped;
    ref_delta_joint_pos_clamped(0) = boost::algorithm::clamp(
        ref_delta_joint_pos(0), delta_min_joint_angle_, delta_max_joint_angle_);
    ref_delta_joint_pos_clamped(1) = boost::algorithm::clamp(
        ref_delta_joint_pos(1), delta_min_joint_angle_, delta_max_joint_angle_);
    ref_delta_joint_pos_clamped(2) = boost::algorithm::clamp(
        ref_delta_joint_pos(2), delta_min_joint_angle_, delta_max_joint_angle_);
    //    std::cout << "ref_delta_joint_pos: " << ref_delta_joint_pos <<
    //    std::endl; std::cout << "ref_delta_joint_pos_clamped: " <<
    //    ref_delta_joint_pos_clamped
    //              << std::endl;
    //    std::cout << "base link idx: " <<
    //    ouzel_->getBodyIdx("ouzel/base_link") << std::endl;
    //    Eigen::Vector3d levitation_force_W(0, 0, ouzel_->getTotalMass()
    //    * 9.81); Eigen::Vector3d levitation_force_B =
    //    ouzel_orientation_W_B_gt_.inverse().toRotationMatrix() *
    //    levitation_force_W; Eigen::Vector3d test_torque_W(0, 0, 0.1);
    //    Eigen::Vector3d test_torque_B =
    //    ouzel_orientation_W_B_gt_.inverse().toRotationMatrix() *
    //    test_torque_W;
    //    Eigen::Vector3d force_B_old(0, 0, 0);
    //    Eigen::Vector3d torque_B_old(0, 0, 0);
    Eigen::VectorXd odometry_measurement_gt;
    Eigen::Vector3d force_B, torque_B;
    Eigen::Vector3d ee_pos, ee_vel, ee_acc, eef_pos_W;
    Eigen::Vector3d B_dv_WB, B_om_WB, B_dom_WB;
    for (int i = 0; i < int(control_dt_ / simulation_dt_ + 1e-10); i++) {
      // get current gt measurements
      //      ouzel_->getState(gc_, gv_);
      //      if (!Eigen::isfinite(gc_.array()).all()) {
      //        std::cout << "gc is nan in for loop of step!! " << i <<
      //        std::endl; std::cout << "odometry : " << odometry_measurement_gt
      //        << std::endl; std::cout << "gc : " << gc_ << std::endl;
      //        std::cout << "gv : " << gv_ << std::endl;
      //        std::cout << "wrench_command.thrust: " << wrench_command.thrust
      //                  << std::endl;
      //        std::cout << "wrench_command.torque: " << wrench_command.torque
      //                  << std::endl;
      //        std::cout << "force_B: " << force_B << std::endl;
      //        std::cout << "torque_B: " << torque_B << std::endl;
      //        //        std::cout << "gv : " << gv_ << std::endl;
      //        raise(SIGTERM);
      //      }
      odometry_.update();
      odometry_measurement_gt = odometry_.getMeasGT();
      //      ouzel_->getState(gc_, gv_);
      //      if (!Eigen::isfinite(gc_.array()).all()) {
      //        std::cout << "gc is nan" << std::endl;
      //        std::cout << "odom gt is nan: " <<
      //        previous_odometry_measurement_gt_
      //                  << std::endl;
      //        std::cout << "commanded thrust:\n"
      //                  << wrench_command.thrust << "\n"
      //                  << "commanded torque:\n"
      //                  << wrench_command.torque << std::endl;
      //        std::cout << "action ref pos: " <<
      //        delta_ouzel_ref_position_offset
      //                  << " action ref pos norm: "
      //                  << delta_ouzel_ref_position_offset.norm() <<
      //                  std::endl;
      //        std::cout << "action ref orient coeffs: "
      //                  << ref_ouzel_orientation_corr.coeffs()
      //                  << " action ref orient angle: "
      //                  <<
      //                  Eigen::AngleAxisd(ref_ouzel_orientation_corr).angle()
      //                  << std::endl;
      //      }
      //      previous_odometry_measurement_gt_ = odometry_measurement_gt;
      ouzel_position_W_gt_ = odometry_measurement_gt.segment(0, 3);
      ouzel_orientation_W_B_gt_ = Eigen::Quaterniond(odometry_measurement_gt(3),
                                                     odometry_measurement_gt(4),
                                                     odometry_measurement_gt(5),
                                                     odometry_measurement_gt(6))
                                      .normalized();
      ouzel_linear_vel_B_gt_ = odometry_measurement_gt.segment(7, 3);
      ouzel_angular_vel_B_gt_ = odometry_measurement_gt.segment(10, 3);

      // apply wrench on ouzel
      ouzel_->setExternalForce(baseLink_, ouzel_->BODY_FRAME,
                               wrench_command.thrust, ouzel_->WORLD_FRAME,
                               ouzel_->getCOM()); // set force in body frame
      ouzel_->setExternalTorqueInBodyFrame(baseLink_, wrench_command.torque);

      // delta arm simulation
      delta_sym_->sendActuatorsCommand(ref_delta_joint_pos_clamped);

      delta_sym_->fwkinPosition(&ee_pos);
      //      std::cout << "delta_pos_ in delta frame: " << ee_pos << std::endl;
      delta_sym_->fwkinVel(&ee_vel, ee_pos);
      ee_acc = (ee_vel - ee_vel_prev_) / simulation_dt_;
      ee_vel_prev_ = ee_vel;

      B_dv_WB =
          ouzel_orientation_W_B_gt_.toRotationMatrix() * ouzel_linear_vel_B_gt_;
      B_om_WB = ouzel_orientation_W_B_gt_.toRotationMatrix() *
                ouzel_angular_vel_B_gt_;
      B_dom_WB = (B_om_WB - B_om_WB_prev_) / simulation_dt_;
      B_om_WB_prev_ = B_om_WB;

      // Compute feed forward base wrench due to dynamics.
      //      Eigen::Vector3d force_B, torque_B;
      delta_sym_->getBaseWrench(&force_B, &torque_B, ouzel_orientation_W_B_gt_,
                                B_om_WB, B_dom_WB, B_dv_WB, ee_pos, ee_vel,
                                ee_acc);
      ouzel_->setExternalForce(baseLink_, ouzel_->BODY_FRAME, force_B,
                               ouzel_->WORLD_FRAME, ouzel_->getCOM());
      ouzel_->setExternalTorqueInBodyFrame(baseLink_, torque_B);

      //      if (!Eigen::isfinite(odometry_measurement_gt.array()).all()) {
      //        std::cout << "delta force B: " << force_B_old << std::endl;
      //        std::cout << "delta torque B: " << torque_B_old << std::endl;
      //      }
      //      force_B_old = force_B;
      //      torque_B_old = torque_B;

      if (visualizable_ && render_) {
        eef_pos_W = ouzel_position_W_gt_ + pos_offset_BD_ +
                    ouzel_orientation_W_B_gt_.matrix() *
                        ang_offset_BD_.matrix() * ee_pos;
        //        std::cout << "relative delta_position_W_gt_: "
        //                  << eef_pos_W - ouzel_position_W_gt_ << std::endl;
        delta_eef_->setPosition(eef_pos_W(0), eef_pos_W(1), eef_pos_W(2));
        //      std::cout << "eef_pos_W: " << eef_pos_W << std::endl;
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

    // get observations
    //    ouzel_->getState(gc_, gv_);
    //    if (!Eigen::isfinite(gc_.array()).all()) {
    //      std::cout << "gc is nan in end of step!!" << std::endl;
    //      //      std::cout << "odometry : " << odometry_measurement <<
    //      std::endl;
    //      //      std::cout << "gc : " << gc_ << std::endl;
    //      //      std::cout << "gv : " << gv_ << std::endl;
    //      raise(SIGTERM);
    //    }

    step_count_ += 1;
    if (step_count_ % int(ref_sampling_time_ / control_dt_ + 0.0001) == 0)
      updateReference();
    updateObservation();

    Eigen::Vector3d delta_ouzel_ref_position_offset_diff;
    if (delta_ouzel_ref_position_offset_previous_ == Eigen::Vector3d::Zero()) {
      delta_ouzel_ref_position_offset_diff = Eigen::Vector3d::Zero();
    } else {
      delta_ouzel_ref_position_offset_diff =
          delta_ouzel_ref_position_offset -
          delta_ouzel_ref_position_offset_previous_;
    }
    delta_ouzel_ref_position_offset_previous_ = delta_ouzel_ref_position_offset;

    Eigen::Vector3d ref_delta_joint_pos_diff;
    if (ref_delta_joint_pos_previous_ == Eigen::Vector3d::Zero()) {
      ref_delta_joint_pos_diff = Eigen::Vector3d::Zero();
    } else {
      ref_delta_joint_pos_diff =
          ref_delta_joint_pos - ref_delta_joint_pos_previous_;
    }
    ref_delta_joint_pos_previous_ = ref_delta_joint_pos_diff;

    // get rewards
    double waypoint_dist_delta, error_angle;
    computeErrorMetrics(waypoint_dist_delta, error_angle);
    Eigen::AngleAxisd ref_ouzel_orientation_corr_angle_axis(
        ref_ouzel_orientation_corr);
    //    std::cout << "waypoint dist: " << waypoint_dist_delta << std::endl;
    //    std::cout << "angle error deg: " << error_angle / M_PI * 180 <<
    //    std::endl; std::cout << "angle error: " << error_angle << std::endl;
    //    std::cout << "ref orient corr angle: " <<
    //    float(ref_ouzel_orientation_corr_angle_axis.angle()) << std::endl;
    rewards_.record("waypointDist", float(waypoint_dist_delta));
    rewards_.record("orientError", float(error_angle));
    rewards_.record("deltaOuzelRefPositionOffset",
                    float(delta_ouzel_ref_position_offset.squaredNorm()));
    rewards_.record("deltaOuzelRefPositionOffsetDiff",
                    float(delta_ouzel_ref_position_offset_diff.squaredNorm()));
    rewards_.record("orientRefCorr",
                    float(ref_ouzel_orientation_corr_angle_axis.angle()));
    rewards_.record("deltaJointAngles",
                    float(ref_delta_joint_pos.squaredNorm()));
    //                    float(std::abs(ref_delta_joint_pos(0)) +
    //                          std::abs(ref_delta_joint_pos(1)) +
    //                          std::abs(ref_delta_joint_pos(2))));
    rewards_.record("deltaJointAnglesDiff",
                    float(ref_delta_joint_pos_diff.squaredNorm()));
    rewards_.record(
        "deltaJointAnglesClamp",
        float((ref_delta_joint_pos - ref_delta_joint_pos_clamped).norm()));
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
      //      Eigen::Vector4d quat(
      //          ref_ouzel_orientation_.w(), ref_ouzel_orientation_.x(),
      //          ref_ouzel_orientation_.y(), ref_ouzel_orientation_.z());
      //      delta_ee_ref_marker_->setOrientation(quat);
    }
    controller_.setRef(ref_delta_position_, ref_ouzel_orientation_);
    //    std::cout << "ref_delta_position_: " << ref_delta_position_ <<
    //    std::endl;
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
    //    std::cout << "delta_joint_angle_: " << delta_joint_angle_ <<
    //    std::endl;

    Eigen::Vector3d end_effector_position_D;
    delta_sym_->fwkinPosition(&end_effector_position_D);
    //    std::cout << "delta pos in B frame: " << end_effector_position_D
    //              << std::endl;
    delta_position_W_ = ouzel_position_W_ + pos_offset_BD_ +
                        ouzel_orientation_W_B_.matrix() *
                            ang_offset_BD_.matrix() * end_effector_position_D;
    delta_position_W_gt_ = ouzel_position_W_gt_ + pos_offset_BD_ +
                           ouzel_orientation_W_B_gt_.matrix() *
                               ang_offset_BD_.matrix() *
                               end_effector_position_D;
    ouzel_->getState(gc_, gv_);

    //    std::cout << "odometry_measurement: " << odometry_measurement <<
    //    std::endl; std::cout << "ouzel position W: " << ouzel_position_W_ <<
    //    std::endl; std::cout << "delta position W: " << delta_position_W_ <<
    //    std::endl;
    //        std::cout << "ouzel_orientation_W_B_ coeffs: " <<
    //    ouzel_orientation_W_B_.coeffs()
    //    << std::endl; std::cout << "ouzel_linear_vel_B_: " <<
    //    ouzel_linear_vel_B_ << std::endl; std::cout << "ouzel_angular_vel_B_:
    //    " << ouzel_angular_vel_B_ << std::endl;

    //    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    //    std::cout << "odometry_measurement gt: " << odometry_measurement_gt <<
    //    std::endl;

    //    std::cout << "delta_joint_angle_: " << delta_joint_angle_ <<
    //    std::endl; std::cout << "delta_joint_angular_vel_: " <<
    //    delta_joint_angular_vel_
    //              << std::endl;

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
        delta_joint_angle_, delta_joint_angular_vel_;
    ob = ob_double.cast<float>();
    if (!Eigen::isfinite(ob.array()).all()) {
      std::cout << "ob is nan: " << ob << std::endl;
    }
    //    std::cout << "ob_double_: " << ob_double << std::endl;
    //    std::cout << "ob: " << ob << std::endl;
    //    std::cout << "ouzel_orientation_W_B_ coeffs: \n" <<
    //    ouzel_orientation_W_B_.coeffs() << std::endl; std::cout <<
    //    "ouzel_orientation_W_B_mat: \n" << ouzel_orientation_W_B_mat <<
    //    std::endl; std::cout << "ouzel_orientation_W_B_mat: " <<
    //    ouzel_orientation_W_B_mat << std::endl; std::cout << "ob: " << ob <<
    //    std::endl;
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
      //      std::cout << "termination oob" << std::endl;
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
      //      std::cout << "termination success" << std::endl;
      //      std::cout << "waypoint_dist_delta: " <<
      //      waypoint_dist_delta <<
      //      std::endl; std::cout << "error_angle: " << error_angle <<
      //      std::endl;
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
    //    init_position << 0.0, 0.0, 0.3;
    ouzel_position_W_ = init_position_;

    Eigen::Quaterniond init_quaternion = Eigen::Quaterniond::Identity();
    ouzel_orientation_W_B_ = init_quaternion;

    Eigen::Vector3d init_lin_vel_dir(unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_),
                                     unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_lin_vel = init_lin_vel_dir.normalized() *
                                   initialLinearVel_ *
                                   unifDistPlusMinusOne_(gen_);
    //    init_lin_vel = Eigen::Vector3d(1.0, 0, 0);
    ouzel_linear_vel_B_ = init_lin_vel;
    Eigen::Vector3d init_lin_vel_W =
        ouzel_orientation_W_B_.toRotationMatrix() * init_lin_vel;
    //    std::cout << "init_lin_vel_W: " << init_lin_vel_W << std::endl;

    Eigen::Vector3d init_ang_vel_axis(unifDistPlusMinusOne_(gen_),
                                      unifDistPlusMinusOne_(gen_),
                                      unifDistPlusMinusOne_(gen_));
    Eigen::Vector3d init_ang_vel = init_ang_vel_axis.normalized() *
                                   initialAngularVel_ *
                                   unifDistPlusMinusOne_(gen_);
    ouzel_angular_vel_B_ = init_ang_vel;
    Eigen::Vector3d init_ang_vel_W =
        ouzel_orientation_W_B_.toRotationMatrix() * init_ang_vel;
    //    std::cout << "init_ang_vel_W: " << init_ang_vel_W << std::endl;

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
    //    init_joint_angles << 0.5, 0.5, 0.5;
    delta_sym_->setJointAngles(init_joint_angles);

    linear_dir_ = int(std::abs(unifDistPlusMinusOne_(gen_)) * 2.999);
    bool oscillation_dir_proxy = std::signbit(unifDistPlusMinusOne_(gen_));
    std::vector<int> all_oscillation_dir{2, 0, 1, 2, 0};
    oscillation_dir_ =
        all_oscillation_dir[linear_dir_ + 1 + int(oscillation_dir_proxy) -
                            int(!oscillation_dir_proxy)];

    sinus_amplitude_ =
        std::abs(unifDistPlusMinusOne_(gen_)) * max_sinus_amplitude_;
    //    lateral_speed_ = max_lateral_speed_;
    lateral_speed_ = std::abs(unifDistPlusMinusOne_(gen_)) * max_lateral_speed_;
    double sinus_wavelength =
        std::abs(unifDistPlusMinusOne_(gen_)) * max_sinus_wavelength_;
    sinus_offset_ = 0.0;
    //    sinus_offset_ = unifDistPlusMinusOne_(gen_) * M_PI;
    bool lateral_direction_proxy = std::signbit(unifDistPlusMinusOne_(gen_));
    lateral_direction_ =
        int(lateral_direction_proxy) - int(!lateral_direction_proxy);
    sinus_angular_freq_ = 2 * M_PI * lateral_speed_ / sinus_wavelength;
    step_count_ = 0;

    ref_delta_position_ = ouzel_position_W_;
    ref_ouzel_orientation_ = ouzel_orientation_W_B_;
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
    //    std::cout << "ref_delta_angle deg: " << ref_delta_angle / M_PI * 180.0
    //    << std::endl; std::cout << "ouzel_orientation_W_B_ coeffs: " <<
    //    ouzel_orientation_W_B_.coeffs() << std::endl; std::cout << "init
    //    ouzel_orientation_W_B_ coeffs: " << ouzel_orientation_W_B_.coeffs() <<
    //    std::endl; std::cout << "ref_delta_quaternion coeffs: " <<
    //    ref_delta_quaternion.coeffs() << std::endl; std::cout <<
    //    "ref_quaternion coeffs: " << ref_quaternion.coeffs() << std::endl;
    //    double error_angle =
    //    ref_ouzel_orientation_.angularDistance(ouzel_orientation_W_B_);
    //    std::cout << "initial error angle: " << error_angle<< std::endl;
  }

  void computeErrorMetrics(double &waypointDist, double &errorAngle) {
    // Maybe want to clamp the error terms?
    //    Eigen::VectorXd odometry_measurement_gt = odometry_.getMeasGT();
    //    Eigen::Vector3d ouzel_position_W_gt =
    //    odometry_measurement_gt.segment(0, 3);
    waypointDist = (delta_position_W_gt_ - ref_delta_position_).norm();
    //    Eigen::Quaterniond current_quat(gc_[3], gc_[4], gc_[5], gc_[6]);
    //    Eigen::Quaterniond
    //    ouzel_orientation_W_B_gt(odometry_measurement_gt(3),
    //    odometry_measurement_gt(4), odometry_measurement_gt(5),
    //    odometry_measurement_gt(6));
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
  Eigen::VectorXd actionMean_, actionStd_;
  //  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
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
  double delta_min_joint_angle_ = 0.0;
  double delta_max_joint_angle_ = 1.5; // rad
  Eigen::Vector3d delta_position_W_;
  Eigen::Vector3d delta_position_W_gt_;

  int baseLink_;

  float time_in_success_state_;
  float min_time_in_success_state_;

  int linear_dir_;
  int oscillation_dir_;

  double sinus_amplitude_;
  double sinus_wave_number_;
  double lateral_speed_;
  double sinus_angular_freq_;
  double sinus_offset_;
  int lateral_direction_;
  double max_sinus_amplitude_;
  double max_lateral_speed_;
  double max_sinus_wavelength_;
  int update_ref_point_all_n_step_;
  float ref_sampling_time_;
  int step_count_;
  //  Eigen::VectorXd previous_odometry_measurement_gt_;

  //  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
  std::uniform_real_distribution<double> unifDistPlusMinusOne_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

} // namespace raisim
