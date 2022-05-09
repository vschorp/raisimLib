//
// Created by vincent on 17.03.22.
//

#ifndef RAISIM_IMPEDANCE_CONTROL_MODULE_H
#define RAISIM_IMPEDANCE_CONTROL_MODULE_H

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

// Eigen library definitions
//#include <eigen_conversions/eigen_msg.h>
#include "Yaml.hpp"
#include <Eigen/Core>

#include "eigen_mav_msgs.h"

namespace rw_omav_controllers {
static constexpr double kDefaultPositioniGain = 0.0;
static constexpr double kDefaultAttitudeiGain = 0.0;
static constexpr double kDefaultPositionGain = 6.0;
static constexpr double kDefaultVelocityGain = 4.7;
static constexpr double kDefaultAttitudeGain = 3.0;
static constexpr double kDefaultAngularRateGain = 0.52;
static constexpr double kDefaultPositionIntegratorLimit = 1.0;
static constexpr double kDefaultAttitudeIntegratorLimit = 1.0;
static constexpr double kDefaultMaxLinearAccel = 9.81;
static constexpr double kDefaultMaxAngularAccel = 10.0;
static constexpr double kDefaultRangeDMin =
    0.1; ///< Min accepted distance range (close to wall)
static constexpr double kDefaultRangeDMax =
    0.5; ///< Max accepted distance range (away from wall)
static constexpr double kDefaultVirtualMassDecay = 1.0;
static constexpr double kDefaultSensorTimeout =
    1.0; ///< Sensor measurement timeout.
static constexpr double kDefaultPosErrorMax =
    0.0; ///< 0 means position error not limited.

class lowpassWrench {
public:
  lowpassWrench(Eigen::Vector3d initForce, Eigen::Vector3d initTorque) {
    forceState_ = initForce;
    torqueState_ = initTorque;
  }
  void update(Eigen::Vector3d force, Eigen::Vector3d torque, double cutoff_rad,
              double dt) {
    forceState_ += (force - forceState_) * cutoff_rad * dt;
    torqueState_ += (torque - torqueState_) * cutoff_rad * dt;
  }
  Eigen::VectorXd getForce() { return forceState_; }
  Eigen::VectorXd getTorque() { return torqueState_; }

private:
  Eigen::Vector3d forceState_;
  Eigen::Vector3d torqueState_;
};

/// controller parameters
struct ImpedanceControlModuleParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImpedanceControlModuleParameters()
      : use_ext_wrench(0),
        position_gain(Eigen::Vector3d::Ones() * kDefaultPositionGain),
        velocity_gain(Eigen::Vector3d::Ones() * kDefaultVelocityGain),
        position_i_gain(Eigen::Vector3d::Ones() * kDefaultPositioniGain),
        attitude_gain(Eigen::Vector3d::Ones() * kDefaultAttitudeGain),
        angular_rate_gain(Eigen::Vector3d::Ones() * kDefaultAngularRateGain),
        attitude_i_gain(Eigen::Vector3d::Ones() * kDefaultAttitudeiGain),
        max_linear_accel(kDefaultMaxLinearAccel),
        max_angular_accel(kDefaultMaxAngularAccel),
        position_integrator_limit(kDefaultPositionIntegratorLimit),
        attitude_integrator_limit(kDefaultAttitudeIntegratorLimit),
        range_d_min(kDefaultRangeDMin), range_d_max(kDefaultRangeDMax),
        virtual_mass_decay(kDefaultVirtualMassDecay),
        vmass_wall(1.0), // 1.0 implies no reaction to external wrenches.
        vmass_free(1.0), // < 1.0 complies to wrench, > 1.0 opposes wrench.
        sensor_timeout(kDefaultSensorTimeout), pos_err_max(kDefaultPosErrorMax),
        virtual_mass_inv(Eigen::Vector3d::Ones()),
        virtual_inertia_inv(Eigen::Vector3d::Ones()), force_p_gain(0.1),
        force_i_gain(0.2), use_force_control(false),
        force_integrator_limit(1.0) {}

  /// whether external wrench is used or not
  // 0: no external wrench used
  // 1: estimated external wrench used
  // 2: f/t sensor measured external wrench used
  int use_ext_wrench;

  // controller gains
  Eigen::Vector3d position_gain;
  Eigen::Vector3d velocity_gain;
  Eigen::Vector3d position_i_gain;
  Eigen::Vector3d attitude_gain;
  Eigen::Vector3d angular_rate_gain;
  Eigen::Vector3d attitude_i_gain;

  double max_linear_accel;
  double max_angular_accel;
  double position_integrator_limit;
  double attitude_integrator_limit;
  double range_d_min;        ///< distance limit for adaptive compliance tuning
  double range_d_max;        ///< distance limit for adaptive compliance tuning
  double virtual_mass_decay; ///< how fast virtual mass is changed
  double sensor_timeout;     ///< ensures recent sensor measurement is used
  double vmass_wall;         ///< compliant virtual mass value
  double vmass_free;         ///< disturbance rejection virtual mass value
  double pos_err_max;

  Eigen::Vector3d virtual_mass_inv;
  Eigen::Vector3d virtual_inertia_inv;

  // Force controller
  double force_p_gain;
  double force_i_gain;
  bool use_force_control;
  double force_integrator_limit;
};

/// implements impedance control
class ImpedanceControlModule {
public:
  /// class constructor
  ImpedanceControlModule(const Yaml::Node &cfg);

  /// class destructor
  ~ImpedanceControlModule() {}

  /**
   * \brief compute wrench command based on saved reference
   * \param[out] wrench_command computed wrench command
   */
  virtual void
  calculateWrenchCommand(mav_msgs::EigenTorqueThrust *wrench_command,
                         const double sampling_time);

  void setOdom(const Eigen::Vector3d &_position,
               const Eigen::Quaterniond &_orientation,
               const Eigen::Vector3d &_velocity_body,
               const Eigen::Vector3d &_angular_velocity);

  void setRefFromAction(const Eigen::Vector3d &_position_corr,
                        const Eigen::Quaterniond &_orientation_corr_mat);
  void setRef(const Eigen::Vector3d &_position,
              const Eigen::Quaterniond &_orientation);

private:
  /**
   * \brief overwrite controller parameters with new values
   * \param[in] cfg overwrite control parameters with those from config file.
   */
  void setControllerParameters(const Yaml::Node &cfg);

  /**
   * \brief compute state errors based on current odometry and reference
   * \param[out] position_error_B body frame position error
   * \param[out] velocity_error_B body frame velocity error
   * \param[out] attitude_error_B body frame attitude error
   * \param[out] rate_error_B body frame angular rate error
   */
  void computeStateError(Eigen::Vector3d *position_error_B,
                         Eigen::Vector3d *velocity_error_B,
                         Eigen::Vector3d *attitude_error_B,
                         Eigen::Vector3d *rate_error_B) const;

  /**
   * \brief compute end effector tool error
   * \param[in] tool_position_error_B body frame position error
   * \param[in] tool_velocity_error_B body frame velocity error
   */
  void computeToolError(Eigen::Vector3d *tool_position_error_B,
                        Eigen::Vector3d *tool_velocity_error_B) const;

  /**
   * \brief compute PID controller commands
   * \param[out] linear_error_B computed desired acceleration based on
   * translational errors \param[out] angular_error_B computed desired
   * acceleration based on rotational errors \param[in] position_error_B body
   * frame position error \param[in] velocity_error_B body frame velocity error
   * \param[in] attitude_error_B body frame attitude error
   * \param[in] rate_error_B body frame angular rate error
   * Applies PID controller to provided errors
   */
  void computePIDerror(Eigen::Vector3d *linear_error_B,
                       Eigen::Vector3d *angular_error_B,
                       const Eigen::Vector3d &position_error_B,
                       const Eigen::Vector3d &velocity_error_B,
                       const Eigen::Vector3d &attitude_error_B,
                       const Eigen::Vector3d &rate_error_B);
  /**
   * \brief adds reference acceleration to controller commanded one
   * \param[in,out] linear_error_B pointer to translational desired acceleration
   * \param[in,out] angular_error_B pointer to rotational desired acceleration
   */
  void addFeedFwdAccel(Eigen::Vector3d *linear_error_B,
                       Eigen::Vector3d *angular_error_B);

  /// smoothly adapts virtual mass based on range measurement
  void updateVirtualMass();

  /**
   * \brief apply impedance controller
   * \param[out] impedance_ctrl_lin_B computed translational acceleration
   * \param[out] impedance_ctrl_ang_B computed rotational acceleration
   * \param[in] ext_linear_accel external translational disturbance acceleration
   * \param[in] ext_angular_accel external rotational disturbance acceleration
   * \param[in] linear_error_B previous desired translational acceleration
   * \param[in] angular_error_B previous desired rotational acceleration
   *
   * This uses the virtual mass to update the desired acceleration
   */
  virtual void
  computeImpedanceControlCommand(Eigen::Vector3d *impedance_ctrl_lin_B,
                                 Eigen::Vector3d *impedance_ctrl_ang_B,
                                 const Eigen::Vector3d &ext_linear_accel,
                                 const Eigen::Vector3d &ext_angular_accel,
                                 const Eigen::Vector3d &linear_error_B,
                                 const Eigen::Vector3d &angular_error_B);

  /**
   * \brief compute the force control command
   * \param[out] command computed control action
   * \param[in] integral_error value of the error integral
   * \param[in] ref force reference
   * \param[in] meas force measured
   * \param[in] sample_time control loop sample time
   */
  void computeForceControlCommand(Eigen::Vector3d &command,
                                  Eigen::Vector3d &integral_error,
                                  const Eigen::Vector3d &meas,
                                  const Eigen::Vector3d &ref,
                                  const double &sample_time) const;

  /// resets position and attitude integrators in PID controller
  virtual void resetIntegrators();

  /**
   * \brief updates position and attitude integrators in PID controller
   * \param[in] position_error position error to add to integrator
   * \param[in] attitude_error attitude error to add to integrator
   */
  void updateIntegrators(const Eigen::Vector3d &position_error,
                         const Eigen::Vector3d &attitude_error,
                         const double sampling_time);

  /**
   * \brief computes centrifugal and coriolis terms in controller
   * \param[out] n_lin  computed as: omega x vel
   * \param[out] n_ang  computed as: I^{-1} * (omega x I * omega)
   */
  void computeCoriolisTerm(Eigen::Vector3d *n_coriolis);

  /**
   * \brief scales vector
   * \param[in,out] vec vector to scale
   * \param[in] maxval desired maximum value for vector norm
   */
  void limitVector(Eigen::Vector3d *vec, const double maxval);
  Eigen::Vector3d projectVector(const Eigen::Vector3d &vector_to_project,
                                const Eigen::Vector3d &target_vector);

  /**
   * \brief clamp a to be between min and max
   * \param[in] a value to clamp
   * \param[in] min minimum value
   * \param[in] max maximum value
   * \return returns clamped value
   */
  double clamp(const double &a, const double &min, const double &max) const;

  /// Controller parameters
  ImpedanceControlModuleParameters control_params_;

  /// vehicle frame information
  Eigen::Affine3d T_T_B_;

  Eigen::Vector3d position_error_integrator_;
  Eigen::Vector3d attitude_error_integrator_;

  // common controller variables
  mav_msgs::EigenTrajectoryPoint ref_;
  mav_msgs::EigenOdometry odom_;
  mav_msgs::EigenTorqueThrust wrench_est_;
  mav_msgs::EigenTorqueThrust wrench_sensor_;

  double mass_;
  double gravity_;
  Eigen::Vector3d com_offset_;
  Eigen::Matrix3d inertia_;

  Eigen::Vector3d ref_position_;
  Eigen::Quaterniond ref_quaternion_;

  lowpassWrench *commandFilter_;
  bool initWrenchFilter_;
};

} // namespace rw_omav_controllers

#endif // RAISIM_IMPEDANCE_CONTROL_MODULE_H
