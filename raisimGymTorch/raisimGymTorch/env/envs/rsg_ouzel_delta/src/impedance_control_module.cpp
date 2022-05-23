//
// Created by vincent on 18.03.22.
//

/*
 * Copyright 2020 Karen Bodie, ASL, ETH Zurich, Switzerland
 * Copyright 2020 Maximilian Brunner, ASL, ETH Zurich, Switzerland
 * Copyright 2020 Michael Pantic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or
 * otherwise.
 *
 */

#include "include/impedance_control_module.h"

namespace rw_omav_controllers {

ImpedanceControlModule::ImpedanceControlModule(const Yaml::Node &cfg) {
  setControllerParameters(cfg);
  T_T_B_ = Eigen::Affine3d::Identity();
  initWrenchFilter_ = false;
  resetIntegrators();
}

void ImpedanceControlModule::calculateWrenchCommand(
    mav_msgs::EigenTorqueThrust *wrench_command, const double sampling_time) {
  assert(wrench_command != nullptr);

  //---------------------------------------------//
  //   Compute state error in body-fixed frame.  //
  //---------------------------------------------//
  Eigen::Matrix3d R_W_B = odom_.orientation_W_B.toRotationMatrix();

  Eigen::Vector3d position_error_B, velocity_error_B, attitude_error_B,
      rate_error_B;
  computeStateError(&position_error_B, &velocity_error_B, &attitude_error_B,
                    &rate_error_B);

  // unused tool_error variables
  //      Eigen::Vector3d tool_position_error_B, tool_velocity_error_B;
  //      computeToolError(&tool_position_error_B, &tool_velocity_error_B);

  // Update all integrators.
  //  updateIntegrators(R_W_B * position_error_B, attitude_error_B,
  //  sampling_time);

  //--------------------------------------------------------//
  // Selective Impedance Control for disturbance rejection. //
  //--------------------------------------------------------//

  //      updateVirtualMass(); // no virtual mass adaptation. no range finder

  // Limit error for impedance control if max value is grater than 0
  Eigen::Vector3d position_error_limited_B = position_error_B;
  if (control_params_.pos_err_max > 0.0) {
    limitVector(&position_error_limited_B, control_params_.pos_err_max);
  }

  Eigen::Vector3d linear_error_B, angular_error_B, impedance_ctrl_lin_B,
      impedance_ctrl_ang_B;
  computePIDerror(&linear_error_B, &angular_error_B, position_error_limited_B,
                  velocity_error_B, attitude_error_B, rate_error_B);
  addFeedFwdAccel(&linear_error_B, &angular_error_B);

  // compute external wrench
  mav_msgs::EigenTorqueThrust wrench_ext;
  switch (control_params_.use_ext_wrench) {
  // do not use external wrench
  case (0):
    wrench_ext.thrust = Eigen::Vector3d::Zero();
    wrench_ext.torque = Eigen::Vector3d::Zero();
    break;
    // use estimated external wrench
  case (1):
    wrench_ext = wrench_est_;
    break;
    // use f/t sensor measured external wrench
  case (2):
    wrench_ext = wrench_sensor_;
    break;
  }

  // load estimated wrench
  Eigen::Vector3d ext_linear_accel_ = 1.0 / mass_ * wrench_ext.thrust;
  Eigen::Vector3d ext_angular_accel_ = inertia_.inverse() * wrench_ext.torque;

  computeImpedanceControlCommand(&impedance_ctrl_lin_B, &impedance_ctrl_ang_B,
                                 ext_linear_accel_, ext_angular_accel_,
                                 linear_error_B, angular_error_B);

  // Force control
  static Eigen::Vector3d force_integral_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_ctrl;
  Eigen::Vector3d ref_force_B =
      odom_.orientation_W_B.toRotationMatrix() * ref_.force_W;
  Eigen::Matrix3d selection_mat = Eigen::Matrix3d::Identity();

  computeForceControlCommand(force_ctrl, force_integral_error,
                             wrench_ext.thrust, ref_force_B, sampling_time);

  if (abs(ref_force_B(0)) > 1e-4 &&
      control_params_.use_force_control) { // Force control active
    selection_mat(0, 0) = 0;
  } else { // Force control disabled
    force_integral_error = Eigen::Vector3d::Zero();
  }
  impedance_ctrl_lin_B =
      selection_mat * impedance_ctrl_lin_B +
      (Eigen::Matrix3d::Identity() - selection_mat) * force_ctrl;

  Eigen::Vector3d n_coriolis;
  computeCoriolisTerm(&n_coriolis);

  // Desired linear and angular acceleration in the geometric base frame
  // without gravity.
  Eigen::Vector3d linear_accel_B_des, angular_accel_B_des;
  linear_accel_B_des = impedance_ctrl_lin_B;
  angular_accel_B_des = impedance_ctrl_ang_B + n_coriolis;

  // Limit desired accelerations before gravity compensation (always
  // compensate for gravity).
  limitVector(&linear_accel_B_des, control_params_.max_linear_accel);
  limitVector(&angular_accel_B_des, control_params_.max_angular_accel);

  // Compensate acceleration due to gravity.
  linear_accel_B_des += R_W_B.transpose() * gravity_ * Eigen::Vector3d::UnitZ();

  // Translate control center from CoM to center of geometry. (add torque
  // due to COM offset)
  Eigen::Vector3d moment_ff = mass_ * com_offset_.cross(linear_accel_B_des);
  angular_accel_B_des += inertia_.inverse() * moment_ff;

  // Init filter with first received commands
  if (!initWrenchFilter_) {
    commandFilter_ = new lowpassWrench(linear_accel_B_des, angular_accel_B_des);
    initWrenchFilter_ = true;
  }
  // Filter with 40.0rad cutoff frequency. Filter runs at 200hz (same sampling
  // rate of the controller)
  commandFilter_->update(linear_accel_B_des, angular_accel_B_des,
                         lowPassFilterCutOffFrequency_, sampling_time);

  (*wrench_command).thrust = mass_ * commandFilter_->getForce();
  (*wrench_command).torque = inertia_ * commandFilter_->getTorque();
}

void ImpedanceControlModule::computeForceControlCommand(
    Eigen::Vector3d &command, Eigen::Vector3d &integral_error,
    const Eigen::Vector3d &meas, const Eigen::Vector3d &ref,
    const double &sample_time) const {
  double Kp = control_params_.force_p_gain;
  double Ki = control_params_.force_i_gain;
  double integ_sat = control_params_.force_integrator_limit;

  integral_error += (meas - ref) * sample_time;

  for (size_t i = 0; i < 3; i++) {
    integral_error(i) = clamp(integral_error(i), -integ_sat, integ_sat);
  }

  command = Kp * (meas - ref) + Ki * integral_error - ref;
  command /= mass_;
}

void ImpedanceControlModule::setControllerParameters(const Yaml::Node &cfg) {
  control_params_.use_ext_wrench =
      int(cfg["use_external_wrench"].template As<float>());

  // translational controller gains
  Eigen::Vector3d linear_axis_gains(cfg["x_gain"].template As<float>(),
                                    cfg["y_gain"].template As<float>(),
                                    cfg["z_gain"].template As<float>());
  control_params_.position_gain =
      cfg["lin_p_gain"].template As<float>() * linear_axis_gains;
  control_params_.velocity_gain =
      cfg["lin_d_gain"].template As<float>() * linear_axis_gains;
  control_params_.position_i_gain =
      cfg["lin_i_gain"].template As<float>() * linear_axis_gains;

  // rotational controller gains
  Eigen::Vector3d angular_axis_gains(cfg["roll_gain"].template As<float>(),
                                     cfg["pitch_gain"].template As<float>(),
                                     cfg["yaw_gain"].template As<float>());
  control_params_.attitude_gain =
      cfg["ang_p_gain"].template As<float>() * angular_axis_gains;
  control_params_.angular_rate_gain =
      cfg["ang_d_gain"].template As<float>() * angular_axis_gains;
  control_params_.attitude_i_gain =
      cfg["ang_i_gain"].template As<float>() * angular_axis_gains;

  control_params_.max_linear_accel =
      cfg["max_linear_accel"].template As<float>();
  control_params_.max_angular_accel =
      cfg["max_angular_accel"].template As<float>();

  control_params_.position_integrator_limit =
      cfg["position_integrator_limit"].template As<float>();
  control_params_.attitude_integrator_limit =
      cfg["attitude_integrator_limit"].template As<float>();

  control_params_.pos_err_max = cfg["pos_err_max"].template As<float>();

  // Selective Impedance Control Parameters
  control_params_.vmass_wall = cfg["tool_virtual_mass"].template As<float>();
  control_params_.vmass_free = cfg["base_virtual_mass"].template As<float>();

  if (cfg["base_virtual_inertia"].template As<float>() <= 0.0) {
    control_params_.virtual_inertia_inv = Eigen::Vector3d::Ones();
  } else {
    control_params_.virtual_inertia_inv =
        (1.0 / cfg["base_virtual_inertia"].template As<float>()) *
        Eigen::Vector3d::Ones();
  }

  control_params_.range_d_min = cfg["range_d_min"].template As<float>();
  control_params_.range_d_max = cfg["range_d_max"].template As<float>();
  control_params_.virtual_mass_decay =
      cfg["virtual_mass_decay"].template As<float>();

  // Force controller params
  control_params_.force_integrator_limit =
      cfg["force_integrator_limit"].template As<float>();
  control_params_.force_p_gain = cfg["force_p_gain"].template As<float>();
  control_params_.force_i_gain = cfg["force_i_gain"].template As<float>();
  control_params_.use_force_control =
      cfg["use_force_control"].template As<bool>();

  // vehicle params
  mass_ = cfg["vehicle_params"]["mass"].template As<float>();
  gravity_ = cfg["vehicle_params"]["gravity"].template As<float>();
  com_offset_ = Eigen::Vector3d(
      cfg["vehicle_params"]["com_offset"]["x"].template As<float>(),
      cfg["vehicle_params"]["com_offset"]["y"].template As<float>(),
      cfg["vehicle_params"]["com_offset"]["z"].template As<float>());
  inertia_ << cfg["vehicle_params"]["inertia"]["xx"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["xy"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["xz"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["xy"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["yy"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["yz"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["xz"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["yz"].template As<float>(),
      cfg["vehicle_params"]["inertia"]["zz"].template As<float>();
}

void ImpedanceControlModule::computeStateError(
    Eigen::Vector3d *position_error_B, Eigen::Vector3d *velocity_error_B,
    Eigen::Vector3d *attitude_error_B, Eigen::Vector3d *rate_error_B) const {
  assert(position_error_B);
  assert(velocity_error_B);
  assert(attitude_error_B);
  assert(rate_error_B);

  // Compute state error in body frame.
  Eigen::Matrix3d R_W_B = odom_.orientation_W_B.toRotationMatrix();
  Eigen::Matrix3d R_W_B_des = ref_.orientation_W_B.toRotationMatrix();

  *position_error_B = R_W_B.transpose() * (odom_.position_W - ref_.position_W);
  *velocity_error_B = odom_.velocity_B - R_W_B.transpose() * ref_.velocity_W;

  // Attitude error according to lee et al.
  Eigen::Matrix3d attitude_error_matrix =
      0.5 * (R_W_B_des.transpose() * R_W_B - R_W_B.transpose() * R_W_B_des);
  Eigen::Vector3d att_error_B;
  mav_msgs::vectorFromSkewMatrix(attitude_error_matrix, &att_error_B);
  *attitude_error_B = att_error_B;
  *rate_error_B = odom_.angular_velocity_B -
                  R_W_B.transpose() * R_W_B_des * ref_.angular_velocity_W;
}

void ImpedanceControlModule::computeToolError(
    Eigen::Vector3d *tool_position_error_B,
    Eigen::Vector3d *tool_velocity_error_B) const {
  assert(tool_position_error_B);
  assert(tool_velocity_error_B);

  // Compute tool position and velocity error in geometric base frame.
  Eigen::Matrix3d R_W_B = odom_.orientation_W_B.toRotationMatrix();
  Eigen::Matrix3d R_W_B_des = ref_.orientation_W_B.toRotationMatrix();

  Eigen::Vector3d tool_position_W =
      odom_.position_W + R_W_B * T_T_B_.translation();
  Eigen::Vector3d tool_position_W_des =
      ref_.position_W + R_W_B_des * T_T_B_.translation();

  *tool_position_error_B =
      R_W_B.transpose() * (tool_position_W - tool_position_W_des);

  Eigen::Vector3d ang_vel_B = odom_.angular_velocity_B;
  Eigen::Vector3d tool_lin_vel_B =
      odom_.velocity_B + ang_vel_B.cross(T_T_B_.translation());
  Eigen::Vector3d tool_lin_vel_B_des =
      R_W_B.transpose() *
      (ref_.velocity_W + R_W_B_des * ang_vel_B.cross(T_T_B_.translation()));

  *tool_velocity_error_B = tool_lin_vel_B - tool_lin_vel_B_des;
}

void ImpedanceControlModule::computePIDerror(
    Eigen::Vector3d *linear_error_B, Eigen::Vector3d *angular_error_B,
    const Eigen::Vector3d &position_error_B,
    const Eigen::Vector3d &velocity_error_B,
    const Eigen::Vector3d &attitude_error_B,
    const Eigen::Vector3d &rate_error_B) {
  assert(linear_error_B);
  assert(angular_error_B);

  Eigen::Matrix3d R_W_B = odom_.orientation_W_B.toRotationMatrix();

  *linear_error_B =
      control_params_.position_gain.cwiseProduct(position_error_B) +
      control_params_.velocity_gain.cwiseProduct(velocity_error_B) +
      control_params_.position_i_gain.cwiseProduct(R_W_B.transpose() *
                                                   position_error_integrator_);

  *angular_error_B =
      control_params_.attitude_gain.cwiseProduct(attitude_error_B) +
      control_params_.angular_rate_gain.cwiseProduct(rate_error_B) +
      control_params_.attitude_i_gain.cwiseProduct(R_W_B.transpose() *
                                                   attitude_error_integrator_);
}

void ImpedanceControlModule::addFeedFwdAccel(Eigen::Vector3d *linear_error_B,
                                             Eigen::Vector3d *angular_error_B) {
  assert(linear_error_B);
  assert(angular_error_B);

  Eigen::Matrix3d R_W_B = odom_.orientation_W_B.toRotationMatrix();
  Eigen::Matrix3d R_W_B_des = ref_.orientation_W_B.toRotationMatrix();
  Eigen::Vector3d ang_vel_B = odom_.angular_velocity_B;

  // Add feed forward acceleration terms.
  // note: angular_velocity_W is actually angular_velocity_B_des
  *linear_error_B -= R_W_B.transpose() * ref_.acceleration_W;
  *angular_error_B +=
      ang_vel_B.cross(R_W_B.transpose() * R_W_B_des * ref_.angular_velocity_W) -
      R_W_B.transpose() * R_W_B_des * ref_.angular_acceleration_W;
}

void ImpedanceControlModule::updateVirtualMass() {
  // Update the virtual mass based on the range measurement (distance from
  // tool to surface). When a wall is detected along the tool axis, the
  // virtual mass in this direction is reduced when we approach.

  if (control_params_.range_d_max > control_params_.range_d_min &&
      control_params_.range_d_min >= 0.0) {
    // Set distance to maximum range by default (nominal virtual mass).
    double distance_to_surface = control_params_.range_d_max;

    // Check that range measurement is within a bound we care about.
    // Allow for tool frame calibration error as a minimum, and reasonable
    // stopping distance as a maximum.
    //        distance_to_surface =
    //                clamp(measured_range_tool_, control_params_.range_d_min,
    //                      control_params_.range_d_max);

    // Reduce virtual mass if the tool is near the surface, according to
    // cosine function: v_mass_val = 0.5 * (1 - cos((d - d_min) / (d_max -
    // d_min) * pi))

    double dist_val =
        (distance_to_surface - control_params_.range_d_min) /
        (control_params_.range_d_max - control_params_.range_d_min);
    double vmass_tool_val = 0.5 * (1.0 - std::cos(dist_val * M_PI));

    // Scale value between vmass_wall and vmass_free.
    double vmass_tool = vmass_tool_val * (control_params_.vmass_free -
                                          control_params_.vmass_wall) +
                        control_params_.vmass_wall;
    Eigen::Vector3d virtual_mass_inv_new;
    virtual_mass_inv_new << (1.0 / control_params_.vmass_free),
        (1.0 / control_params_.vmass_free), (1.0 / vmass_tool);

    // Smoothly change virtual mass values.
    control_params_.virtual_mass_inv =
        control_params_.virtual_mass_decay * virtual_mass_inv_new +
        (1.0 - control_params_.virtual_mass_decay) *
            control_params_.virtual_mass_inv;
  }
}

void ImpedanceControlModule::computeImpedanceControlCommand(
    Eigen::Vector3d *impedance_ctrl_lin_B,
    Eigen::Vector3d *impedance_ctrl_ang_B,
    const Eigen::Vector3d &ext_linear_accel,
    const Eigen::Vector3d &ext_angular_accel,
    const Eigen::Vector3d &linear_error_B,
    const Eigen::Vector3d &angular_error_B) {
  assert(impedance_ctrl_lin_B);
  assert(impedance_ctrl_ang_B);

  // Diagonal virtual inertia shaping matrices for impedance control.
  // Rotate inertia multipliers into the tool orientation,
  // e.g. (R_BE * m_mult * R_BE^T)^(-1)
  //     = R_BE^T * m_mult_inv * R_BE
  Eigen::Matrix3d virtual_mass_inv =
      T_T_B_.rotation().transpose() *
      control_params_.virtual_mass_inv.asDiagonal() * T_T_B_.rotation();
  Eigen::Matrix3d virtual_inertia_inv =
      T_T_B_.rotation().transpose() *
      control_params_.virtual_inertia_inv.asDiagonal() * T_T_B_.rotation();

  *impedance_ctrl_lin_B =
      (virtual_mass_inv - Eigen::Matrix3d::Identity()) * ext_linear_accel -
      linear_error_B;

  *impedance_ctrl_ang_B =
      (virtual_inertia_inv - Eigen::Matrix3d::Identity()) * ext_angular_accel -
      angular_error_B;
}

void ImpedanceControlModule::resetIntegrators() {
  position_error_integrator_.setZero();
  attitude_error_integrator_.setZero();
}

void ImpedanceControlModule::updateIntegrators(
    const Eigen::Vector3d &position_error,
    const Eigen::Vector3d &attitude_error, const double sampling_time) {
  position_error_integrator_ += position_error * sampling_time;
  attitude_error_integrator_ += attitude_error * sampling_time;

  for (uint i = 0; i < 3; i++) {
    if (std::abs(position_error_integrator_(i)) >
        control_params_.position_integrator_limit) {
      position_error_integrator_(i) =
          std::copysign(control_params_.position_integrator_limit,
                        position_error_integrator_(i));
    }
    if (std::abs(attitude_error_integrator_(i)) >
        control_params_.attitude_integrator_limit) {
      attitude_error_integrator_(i) =
          std::copysign(control_params_.attitude_integrator_limit,
                        attitude_error_integrator_(i));
    }
  }
}

void ImpedanceControlModule::computeCoriolisTerm(Eigen::Vector3d *n_coriolis) {
  assert(n_coriolis);
  Eigen::Vector3d ang_vel_B = odom_.angular_velocity_B;
  // Nonlinear terms (Coriolis).
  *n_coriolis = inertia_.inverse() * (ang_vel_B).cross(inertia_ * ang_vel_B);
}

void ImpedanceControlModule::limitVector(Eigen::Vector3d *vec,
                                         const double maxval) {
  if (vec->norm() > maxval) {
    *vec = vec->normalized() * maxval;
  }
}

Eigen::Vector3d
ImpedanceControlModule::projectVector(const Eigen::Vector3d &vector_to_project,
                                      const Eigen::Vector3d &target_vector) {
  if (target_vector.norm() <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  double projection_scale = vector_to_project.dot(target_vector.normalized());
  return projection_scale * (target_vector.normalized());
}

double ImpedanceControlModule::clamp(const double &a, const double &min,
                                     const double &max) const {
  if (a < min)
    return min;
  if (a > max)
    return max;
  return a;
}

void ImpedanceControlModule::setOdom(const Eigen::Vector3d &_position,
                                     const Eigen::Quaterniond &_orientation,
                                     const Eigen::Vector3d &_velocity_body,
                                     const Eigen::Vector3d &_angular_velocity) {
  odom_.position_W = _position;
  odom_.orientation_W_B = _orientation.normalized();
  odom_.velocity_B = _velocity_body;
  odom_.angular_velocity_B = _angular_velocity;
}

void ImpedanceControlModule::adaptRefFromAction(
    const Eigen::Vector3d &_delta_ouzel_ref_position_offset_W,
    const Eigen::Quaterniond &_ouzel_orientation_corr_quat) {
  ref_.position_W = ref_position_delta_W;
  ref_.position_W += _delta_ouzel_ref_position_offset_W;
  if (!Eigen::isfinite(ref_.position_W.array()).any()) {
    std::cout << "ref position is nan!!" << std::endl;
  }

  ref_.orientation_W_B = ref_quaternion_ouzel_W;
  ref_.orientation_W_B = ref_.orientation_W_B * _ouzel_orientation_corr_quat;
  ref_.orientation_W_B.normalize();
  if (!Eigen::isfinite(ref_.orientation_W_B.toRotationMatrix().array()).any()) {
    std::cout << "ref orientation is nan!!" << std::endl;
  }
}

void ImpedanceControlModule::setRef(
    const Eigen::Vector3d &_ref_delta_position_W,
    const Eigen::Quaterniond &_ref_ouzel_orientation_WB) {
  ref_position_delta_W = _ref_delta_position_W;
  ref_quaternion_ouzel_W = _ref_ouzel_orientation_WB;

  ref_.position_W = ref_position_delta_W;
  ref_.orientation_W_B = ref_quaternion_ouzel_W;
}

} // namespace rw_omav_controllers
