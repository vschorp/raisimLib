/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "ros/ros.h"
//Delta arm
#include <delta_control/controller_helpers.h>
#include <delta_control/dynamics/dynamics.h>
#include <delta_control/kinematics/kinematics.h>
#include "ros_raisim_interface/actuators.h"


#ifndef RAISIM_DELTA_DYNAMICS_
#define RAISIM_DELTA_DYNAMICS_

namespace delta_dynamics {

  static constexpr double kDefaultMassBase = 0.53;
  static constexpr double kDefaultMassDeltaBase = 0.46;
  static constexpr double kDefaultMassPromixalLink = 0.06;
  static constexpr double kDefaultMassPlatform = 0.125;

  const Eigen::Vector3d kDefaultPosBaseToDelta = Eigen::Vector3d(0.0, 0.0, 0.065);
  const Eigen::Quaterniond kDefaultQBaseToDelta = Eigen::Quaterniond::Identity();

  const Eigen::Vector3d kDefaultIntertiaBase = 1e-6 * Eigen::Vector3d(1581.5, 1269.7, 2825.0);
  const Eigen::Vector3d kDefaultIntertiaDeltaBase = 1e-6 * Eigen::Vector3d(1696.4, 1696.4, 3059.1);
  const Eigen::Vector3d kDefaultIntertiaProximalLink = 1e-6 * Eigen::Vector3d(22.0, 801.8, 817.0);
  const Eigen::Vector3d kDefaultIntertiaPlatform = 1e-6 * Eigen::Vector3d(136.8, 143.9, 254.8);

  const Eigen::Vector3d kDefaultComOffsetBase = Eigen::Vector3d(0.0, 0.0, 0.003);
  const Eigen::Vector3d kDefaultComOffsetDeltaBase = Eigen::Vector3d(0.0, 0.0, -0.004);
  const Eigen::Vector3d kDefaultComOffsetProximalLink = Eigen::Vector3d(0.099, 0.0, 0.0);
  const Eigen::Vector3d kDefaultComOffsetPlatform = Eigen::Vector3d(0.0, 0.0, 0.0016);


static constexpr double kDefaultPosIntegratorMax = 0.2;  // [m] compensate for 20cm offset
static constexpr double kDefaultQIntegratorMax = 0.2;    // [rad] compensate for 0.2 rad offset
static constexpr double kDefaultFilterCutoffFrequency = 5.0;
static constexpr double kDefaultFixedArmZ = 0.25;  // [m] from delta base

// Bitmasks and bounds for workspace projection.
const std::vector<std::uint_fast8_t> mask{0b00000110, 0b00000101, 0b00000011};
const std::vector<std::uint_fast8_t> upper_region = {0b00000100, 0b00000000, 0b00000011};
const std::vector<std::uint_fast8_t> lower_region = {0b00000010, 0b00000101, 0b00000000};

class DeltaController {
 public:
  DeltaController(ros::NodeHandle&,double);
  ~DeltaController(){};

  void initialize(const double& r_B, const double& r_T, const double& l_P, const double& l_D,
                  const double& theta_min, const double& theta_max, const double& dt);

  void initializeDynamics(const double& _m_Base, const double& _m_O, const double& _m_A,
                          const double& _m_P, const Eigen::Vector3d& _I_Base,
                          const Eigen::Vector3d& _I_O, const Eigen::Vector3d& _I_A,
                          const Eigen::Vector3d& _I_P, const Eigen::Vector3d& _pCom_Base,
                          const Eigen::Vector3d& _pCom_O, const Eigen::Vector3d& _pCom_A,
                          const Eigen::Vector3d& _pCom_P, const Eigen::Vector3d& _p_BO,
                          const Eigen::Quaterniond& _q_BO);

  void setGains(const double& p_gain, const double& i_gain, const double& d_gain,
                const double& ff_gain);

  void setMasses(const double& mass_base, const double& mass_prox, const double& mass_dist,
                 const double& mass_tool, const double& mass_elbow, const double& mass_motor,
                 const double& radius_motor);

  void updateSmoothing(const double& pos_smoothing, const double& vel_smoothing,
                       const double& eff_smoothing);

  void updateErrorLimits(const double& max_pos_error, const double& max_vel_error);

  void updateIntegrators(const Eigen::Vector3d& pos_error, const Eigen::Vector3d& q_error,
                         const double& dt);

  void resetIntegrators();

  void setMode(int ctrl_mode);

  int getMode() const;

  double pGain() { return params_->p_gain(); };

  double iGain() { return params_->i_gain(); };

  double dGain() { return params_->d_gain(); };

  Eigen::Vector3d getParamVector3(const std::string& param_name, const Eigen::Vector3d& default_value);
  Eigen::Quaterniond getParamQuat(const std::string& param_name, const Eigen::Quaterniond& default_value);

  void deltaCommandCb(const mav_msgs::Actuators &msg);

  void sendActuatorsCommand(const Eigen::Vector3d jointsPos);
  void sendActuatorsCommand();
  void publishJointStates();

  void updateState(const Eigen::Quaterniond& base_q_WB, const Eigen::Vector3d& base_vel_B,
                   const Eigen::Vector3d& base_angvel_B, const Eigen::VectorXd& q_in,
                   const Eigen::VectorXd& qd_in, const Eigen::VectorXd& effort_in,
                   const double& dt);

  void updateState(const Eigen::Quaterniond& base_q_WB, const Eigen::Vector3d& base_vel_B,
                   const Eigen::Vector3d& base_angvel_B, const Eigen::Vector3d& q_in,
                   const Eigen::Vector3d& qd_in, const Eigen::Vector3d& effort_in,
                   const double& dt);

  void updateState(const Eigen::Quaterniond& base_q_WB, const Eigen::Vector3d& base_vel_B,
                   const Eigen::Vector3d& base_angvel_B, const Eigen::Vector3d& q,
                   const Eigen::Vector3d& qd, const Eigen::Vector3d& effort,
                   const Eigen::Vector3d& tool_pos_B, const Eigen::Vector3d& tool_vel_B,
                   const double& dt);

  void updateReference(const Eigen::Vector3d& pos_B, const Eigen::Vector3d& vel_B,
                       const Eigen::Vector3d& acc_B, const Eigen::Vector3d& force_B);

  void getControlCommand(Eigen::VectorXd* delta_pos_cmd, Eigen::VectorXd* delta_vel_cmd,
                         Eigen::VectorXd* delta_eff_cmd, const double& dt);

  void getBaseWrench(Eigen::Vector3d* force_B, Eigen::Vector3d* torque_B);

  void getBaseWrench(Eigen::Vector3d* force_B, Eigen::Vector3d* torque_B, const Eigen::Quaterniond& q_WB,
                     const Eigen::Vector3d& B_om_WB /*angvel*/, const Eigen::Vector3d& B_dom_WB /*angacc*/,
                     const Eigen::Vector3d& B_dv_WB /*linacc*/, const Eigen::Vector3d& p_OE /*tool_pos*/,
                     const Eigen::Vector3d& v_OE /*tool_vel*/, const Eigen::Vector3d& dv_OE /*tool_acc*/); 

  void getTargetPos(Eigen::Vector3d* delta_pos_ref);

  void getWorkspacePositionError(Eigen::Vector3d* tool_pos_err);

  void getToolPosition(Eigen::Vector3d* tool_pos);

  void getJointspacePositionError(Eigen::Vector3d* q_err);

  bool projectToWorkspace(Eigen::Vector3d* projection, const Eigen::Vector3d& point);

  bool fwkinPosition(Eigen::Vector3d* ee_pos, const Eigen::Vector3d& q);
  bool fwkinPosition(Eigen::Vector3d* ee_pos);
  bool fwkinVel(Eigen::Vector3d* ee_vel, const Eigen::Vector3d& ee_pos);

  Eigen::Vector3d getqPos();
  Eigen::Vector3d getqVel();
  Eigen::Vector3d getqAcc();

  void fixArm(const bool set_fixed);

  void setFixedZ(const double& fixed_arm_z);

  void setBaseWrenchFilter(const bool set_filter, const double& cutoff_freq, const double& dt);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  bool inkinPosition(Eigen::Vector3d* q, const Eigen::Vector3d& ee_pos);

  bool inkinVelocity(Eigen::Vector3d* qdot, const Eigen::Vector3d& ee_vel);

  void getWorkspaceRegion(int* upper_index, int* lower_index, const Eigen::Vector3d& point);

  bool projectToSphere(Eigen::Vector3d* projection, const Eigen::Vector3d& point,
                       const Eigen::Vector3d& center, const double& radius, const int direction);

  bool projectToHeight(Eigen::Vector3d* projection, const Eigen::Vector3d& point,
                       const double& height, const int direction);

  double angleWrap(const double& angle, const double& offset);

  Eigen::Vector3d limitVector(Eigen::Vector3d& vec, const double& limit);

  Eigen::Matrix3d RotZ(const double& angle);

  std::shared_ptr<delta_control::DeltaControllerParameters> params_;
  std::shared_ptr<delta_control::Kinematics> kin_;
  std::shared_ptr<delta_control::Dynamics> dyn_;
  std::shared_ptr<delta_control::DeltaState> state_;
  std::shared_ptr<delta_control::DeltaRef> reference_;
  Eigen::Vector3d prev_q_cmd_;

  Eigen::Vector3d pos_error_integrator_;
  Eigen::Vector3d q_error_integrator_;
  double pos_error_integrator_max_;
  double q_error_integrator_max_;
  double filter_cutoff_f_;

  Eigen::Vector3d p_BO_;
  Eigen::Quaterniond q_BO_;

  bool arm_fixed_;
  double fixed_arm_z_;
  bool filter_base_wrench_;
  bool base_wrench_butterworth_initialized_;
  boost::circular_buffer<Eigen::Vector3d> past_base_force_{5};
  boost::circular_buffer<Eigen::Vector3d> past_base_torque_{5};
  Iir::Butterworth::LowPass<2> base_force_butterworth_[3];
  Iir::Butterworth::LowPass<2> base_torque_butterworth_[3];
  std::vector<raisim_actuators::delta_joint*> joints_;
  ros::NodeHandle pnh_;
  ros::Subscriber delta_command_sub_;
  ros::Publisher delta_state_pub_;
  ros::Publisher base_wrench_pub_;
  double sample_time_;
  Eigen::Vector3d joints_command_;
};

}


#endif // !RAISIM_DELTA_DYNAMICS_