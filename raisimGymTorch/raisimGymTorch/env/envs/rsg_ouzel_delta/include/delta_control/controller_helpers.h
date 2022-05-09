/*
 * Copyright 2021 Karen Bodie, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or
 * otherwise.
 *
 */

#ifndef DELTA_CONTROL_CONTROLLER_HELPERS_H
#define DELTA_CONTROL_CONTROLLER_HELPERS_H

#pragma GCC system_header

#include <Eigen/Core>
#include <Eigen/Eigen>

// Filtering and numerical diff
#include "iir1/Iir.h"
#include <boost/circular_buffer.hpp>

// Debug
#include <iostream>

namespace delta_control {

static constexpr double kDefaultBasePlateRadius = 0.075;     // [m]
static constexpr double kDefaultToolPlateRadius = 0.028;     // [m]
static constexpr double kDefaultProximalLinkLength = 0.156;  // [m]
static constexpr double kDefaultDistalLinkLength = 0.243;    // [m]
static constexpr double kDefaultServoMin = -M_PI / 3.0;      // [rad]
static constexpr double kDefaultServoMax = 2.0 / 3.0 * M_PI; // [rad]
static constexpr double kDefaultPGain = 1.0;
static constexpr double kDefaultIGain = 0.01;
static constexpr double kDefaultDGain = 0.1;
static constexpr double kDefaultFeedForwardGain = 1.0;
static constexpr double kDefaultWorkspaceBuffer = 0.01;
static constexpr double kGravity = 9.81;             // [m/s^2]
static constexpr double kDefaultMassBase = 0.45;     // [kg]
static constexpr double kDefaultMassProximal = 0.04; // [kg]
static constexpr double kDefaultMassDistal = 0.02;   // [kg]
static constexpr double kDefaultMassTool = 0.04;     // [kg]
static constexpr double kDefaultMassElbow = 0.004;   // [kg]
static constexpr double kDefaultMassMotor = 0.06;    // [kg]
static constexpr double kDefaultRadiusMotor = 0.01;  // [m]

enum class DeltaControlMode { Position = 0, Velocity = 1 };

class DeltaControllerParameters {
public:
  DeltaControllerParameters()
      : r_B_(kDefaultBasePlateRadius), r_T_(kDefaultToolPlateRadius),
        l_P_(kDefaultProximalLinkLength), l_D_(kDefaultDistalLinkLength),
        theta_min_(kDefaultServoMin), theta_max_(kDefaultServoMax),
        p_gain_(kDefaultPGain), i_gain_(kDefaultIGain), d_gain_(kDefaultDGain),
        ff_gain_(kDefaultFeedForwardGain),
        control_mode_(DeltaControlMode::Position), min_z_(0.0), max_z_(0.0),
        max_pos_error_(0.0), max_vel_error_(0.0),
        ws_buffer_(kDefaultWorkspaceBuffer), g_(kGravity),
        m_B_(kDefaultMassBase), m_P_(kDefaultMassProximal),
        m_D_(kDefaultMassDistal), m_T_(kDefaultMassTool),
        m_E_(kDefaultMassElbow), m_M_(kDefaultMassMotor),
        r_M_(kDefaultRadiusMotor) {
    initializeParams();
  }
  ~DeltaControllerParameters() {}

  void initializeParams() {
    // Configuration parameter "gamma": rotation of arm link about base z-axis.
    gamma_ << 0.0, 2.0 / 3.0 * M_PI, 4.0 / 3.0 * M_PI;
    Eigen::Matrix3d R(Eigen::Matrix3d::Zero());
    Eigen::Vector3d ub(r_B_ - r_T_ + l_P_ * std::cos(theta_max_), 0.0,
                       l_P_ * std::sin(theta_max_));
    Eigen::Vector3d lb(r_B_ - r_T_ + l_P_ * std::cos(theta_min_), 0.0,
                       l_P_ * std::sin(theta_min_));

    for (uint i = 0; i < 3; ++i) {
      R << std::cos(gamma_(i)), -std::sin(gamma_(i)), 0.0, std::sin(gamma_(i)),
          std::cos(gamma_(i)), 0.0, 0.0, 0.0, 1.0;
      ub_sphere[i] = R * ub;
      lb_sphere[i] = R * lb;
    }
    double r_min = l_D_ + ws_buffer_;
    min_z_ = std::sqrt(r_min * r_min - lb(0) * lb(0)) + lb(2);
    double r_max = l_D_ - ws_buffer_;
    max_z_ = std::sqrt(r_max * r_max - ub(0) * ub(0)) + ub(2);

    I_M_ = m_M_ / 2.0 * r_M_ * r_M_;
  }

  void updateParameters(const double &r_B, const double &r_T, const double &l_P,
                        const double &l_D, const double &theta_min,
                        const double &theta_max) {
    r_B_ = r_B;
    r_T_ = r_T;
    l_P_ = l_P;
    l_D_ = l_D;
    theta_min_ = theta_min;
    theta_max_ = theta_max;
    initializeParams();
  }

  void setMasses(const double &mass_base, const double &mass_prox,
                 const double &mass_dist, const double &mass_tool,
                 const double &mass_elbow, const double &mass_motor,
                 const double &radius_motor) {
    m_B_ = mass_base;
    m_P_ = mass_prox;
    m_D_ = mass_dist;
    m_T_ = mass_tool;
    m_E_ = mass_elbow;
    m_M_ = mass_motor;
    r_M_ = radius_motor;
    I_M_ = m_M_ / 2.0 * r_M_ * r_M_;
  }

  void setGains(const double &p_gain, const double &i_gain,
                const double &d_gain, const double &ff_gain) {
    p_gain_ = p_gain;
    i_gain_ = i_gain;
    d_gain_ = d_gain;
    ff_gain_ = ff_gain;
  }

  void updateErrorLimits(const double &max_pos_error,
                         const double &max_vel_error) {
    max_pos_error_ = max_pos_error;
    max_vel_error_ = max_vel_error;
  }

  void setControlMode(const DeltaControlMode &control_mode) {
    control_mode_ = control_mode;
  }

  void getControlMode(DeltaControlMode *control_mode) {
    *control_mode = control_mode_;
  }

  double r_B() { return r_B_; };
  double r_T() { return r_T_; };
  double l_P() { return l_P_; };
  double l_D() { return l_D_; };
  double theta_min() { return theta_min_; };
  double theta_max() { return theta_max_; };
  double p_gain() { return p_gain_; };
  double i_gain() { return i_gain_; };
  double d_gain() { return d_gain_; };
  double ff_gain() { return ff_gain_; };
  double min_z() { return min_z_; };
  double max_z() { return max_z_; };
  double max_pos_error() { return max_pos_error_; };
  double max_vel_error() { return max_vel_error_; };
  double ws_buffer() { return ws_buffer_; };
  double g() { return g_; };
  double m_B() { return m_B_; };
  double m_P() { return m_P_; };
  double m_D() { return m_D_; };
  double m_T() { return m_T_; };
  double m_E() { return m_E_; };
  double m_M() { return m_M_; };
  double r_M() { return r_M_; };
  double I_M() { return I_M_; };
  Eigen::Vector3d gamma() { return gamma_; };
  DeltaControlMode control_mode() { return control_mode_; };

  // Workspace bounding geometry
  std::vector<Eigen::Vector3d> ub_sphere{Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> lb_sphere{Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero()};

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Kinematic params.
  double r_B_;
  double r_T_;
  double l_P_;
  double l_D_;
  double theta_min_;
  double theta_max_;

  // Control gains.
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double ff_gain_;

  double max_pos_error_;
  double max_vel_error_;

  // Worskpace params.
  double min_z_;
  double max_z_;
  double ws_buffer_;

  // Dynamic params.
  double g_;
  // Masses [kg]
  double m_B_;
  double m_P_;
  double m_D_;
  double m_T_;
  double m_E_; // elbow mass
  // Motor inertia (cylinder)
  double m_M_;
  double r_M_;
  double I_M_;

  DeltaControlMode control_mode_;
  Eigen::Vector3d gamma_;
};

class DeltaState {
public:
  DeltaState()
      : q_(Eigen::Vector3d::Zero()), qd_(Eigen::Vector3d::Zero()),
        qdd_(Eigen::Vector3d::Zero()), effort_(Eigen::Vector3d::Zero()),
        tool_pos_B_(Eigen::Vector3d::Zero()),
        tool_vel_B_(Eigen::Vector3d::Zero()),
        tool_acc_B_(Eigen::Vector3d::Zero()),
        base_q_WB_(Eigen::Quaterniond::Identity()),
        base_angvel_WB_(Eigen::Vector3d::Zero()),
        base_angacc_WB_(Eigen::Vector3d::Zero()),
        base_vel_WB_(Eigen::Vector3d::Zero()),
        base_acc_WB_(Eigen::Vector3d::Zero()), butterworth_initialized_(false) {
  }
  ~DeltaState() {}

  Eigen::Vector3d q() { return q_; }
  Eigen::Vector3d qd() { return qd_; }
  Eigen::Vector3d qdd() { return qdd_; }
  Eigen::Vector3d effort() { return effort_; }
  Eigen::Vector3d tool_pos_B() { return tool_pos_B_; }
  Eigen::Vector3d tool_vel_B() { return tool_vel_B_; }
  Eigen::Vector3d tool_acc_B() { return tool_acc_B_; }
  Eigen::Quaterniond base_q_WB() { return base_q_WB_; }
  Eigen::Vector3d base_angvel_WB() { return base_angvel_WB_; }
  Eigen::Vector3d base_angacc_WB() { return base_angacc_WB_; }
  Eigen::Vector3d base_vel_WB() { return base_vel_WB_; }
  Eigen::Vector3d base_acc_WB() { return base_acc_WB_; }

  void update(const Eigen::Vector3d &_q, const Eigen::Vector3d &_qd,
              const Eigen::Vector3d &_effort, const double &dt) {
    q_ = _q;
    qd_ = _qd;
    effort_ = _effort;

    if (butterworth_initialized_ && !qd_.hasNaN()) {
      for (int i = 0; i < 3; i++) {
        filtered_qd_(i) = qd_butterworth_[i].filter(qd_(i));
      }
      past_qd_.push_back(filtered_qd_);
      numericalDerivative(past_qd_, dt, &qdd_);
    }
  }

  void updateBase(const Eigen::Quaterniond &_base_q_WB,
                  const Eigen::Vector3d &_base_vel_B,
                  const Eigen::Vector3d &_base_angvel_B, const double &dt) {
    base_q_WB_ = _base_q_WB;
    base_vel_WB_ = _base_vel_B;
    base_angvel_WB_ = _base_angvel_B;
    if (butterworth_initialized_ && !base_vel_WB_.hasNaN() &&
        !base_angvel_WB_.hasNaN()) {
      for (int i = 0; i < 3; i++) {
        filtered_base_vel_(i) =
            base_vel_butterworth_[i].filter(base_vel_WB_(i));
        filtered_base_angvel_(i) =
            base_angvel_butterworth_[i].filter(base_angvel_WB_(i));
      }
      past_base_vel_.push_back(filtered_base_vel_);
      past_base_angvel_.push_back(filtered_base_angvel_);
      numericalDerivative(past_base_vel_, dt, &base_acc_WB_);
      numericalDerivative(past_base_angvel_, dt, &base_angacc_WB_);
    }
  }

  void updateTool(const Eigen::Vector3d &_tool_pos_B,
                  const Eigen::Vector3d &_tool_vel_B, const double &dt) {
    tool_pos_B_ = _tool_pos_B;
    tool_vel_B_ = _tool_vel_B;

    if (butterworth_initialized_ && !tool_vel_B_.hasNaN()) {
      for (int i = 0; i < 3; i++) {
        filtered_tool_vel_(i) = tool_vel_butterworth_[i].filter(tool_vel_B_(i));
      }
      past_tool_vel_.push_back(filtered_tool_vel_);
      numericalDerivative(past_tool_vel_, dt, &tool_acc_B_);
    }
  }

  void setupButterworthFilters(const double &sampling_rate,
                               const double &filter_cutoff_f) {
    for (int i = 0; i < 3; i++) {
      qd_butterworth_[i].setup(sampling_rate, filter_cutoff_f);
      tool_vel_butterworth_[i].setup(sampling_rate, filter_cutoff_f);
      base_vel_butterworth_[i].setup(sampling_rate, filter_cutoff_f);
      base_angvel_butterworth_[i].setup(sampling_rate, filter_cutoff_f);
    }
    butterworth_initialized_ = true;
  }

  void
  numericalDerivative(const boost::circular_buffer<Eigen::Vector3d> &values,
                      const double &dt, Eigen::Vector3d *result) {
    // Uses 3-5 points to compute numerical derivative
    // Coefficients taken from
    // http://web.media.mit.edu/~crtaylor/calculator.html
    const int n = 4;
    int n_diff = 4;
    if (n_diff == 3) {
      *result = (values[n - 2] - 4 * values[n - 1] + 3 * values[n]) / (2 * dt);
    } else if (n_diff == 4) {
      *result = (-2 * values[n - 3] + 9 * values[n - 2] - 18 * values[n - 1] +
                 11 * values[n + 0]) /
                (6 * dt);
    } else if (n_diff == 5) {
      *result = (3 * values[n - 4] - 16 * values[n - 3] + 36 * values[n - 2] -
                 48 * values[n - 1] + 25 * values[n + 0]) /
                (12 * dt);
    }
  }

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d q_;
  Eigen::Vector3d qd_;
  Eigen::Vector3d qdd_;
  Eigen::Vector3d effort_;
  Eigen::Vector3d tool_pos_B_;
  Eigen::Vector3d tool_vel_B_;
  Eigen::Vector3d tool_acc_B_;
  Eigen::Quaterniond base_q_WB_;
  Eigen::Vector3d base_angvel_WB_;
  Eigen::Vector3d base_angacc_WB_;
  Eigen::Vector3d base_vel_WB_;
  Eigen::Vector3d base_acc_WB_;

  Eigen::Vector3d filtered_qd_;
  Eigen::Vector3d filtered_base_vel_;
  Eigen::Vector3d filtered_base_angvel_;
  Eigen::Vector3d filtered_tool_vel_;

  bool butterworth_initialized_;
  boost::circular_buffer<Eigen::Vector3d> past_qd_{5};
  boost::circular_buffer<Eigen::Vector3d> past_tool_vel_{5};
  boost::circular_buffer<Eigen::Vector3d> past_base_vel_{5};
  boost::circular_buffer<Eigen::Vector3d> past_base_angvel_{5};
  Iir::Butterworth::LowPass<2> qd_butterworth_[3];
  Iir::Butterworth::LowPass<2> tool_vel_butterworth_[3];
  Iir::Butterworth::LowPass<2> base_vel_butterworth_[3];
  Iir::Butterworth::LowPass<2> base_angvel_butterworth_[3];
};

class DeltaRef {
public:
  DeltaRef()
      : pos_(Eigen::Vector3d::Zero()), vel_(Eigen::Vector3d::Zero()),
        acc_(Eigen::Vector3d::Zero()), force_(Eigen::Vector3d::Zero()) {}
  ~DeltaRef() {}

  Eigen::Vector3d pos() { return pos_; }
  Eigen::Vector3d vel() { return vel_; }
  Eigen::Vector3d acc() { return acc_; }
  Eigen::Vector3d force() { return force_; }

  void update(const Eigen::Vector3d &_pos, const Eigen::Vector3d &_vel,
              const Eigen::Vector3d &_acc, const Eigen::Vector3d &_force) {
    pos_ = _pos;
    vel_ = _vel;
    acc_ = _acc;
    force_ = _force;
  }

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  Eigen::Vector3d force_;
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d mat;
  mat.setZero();
  mat(0, 1) = -vec(2);
  mat(0, 2) = vec(1);
  mat(1, 2) = -vec(0);

  mat(1, 0) = vec(2);
  mat(2, 0) = -vec(1);
  mat(2, 1) = vec(0);
  return mat;
}

} // namespace delta_control

#endif DELTA_CONTROL_CONTROLLER_HELPERS_H
