/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "include/delta_dynamics.h"
//#include "mav_msgs/Actuators.h"

namespace delta_dynamics {

DeltaController::DeltaController(const Yaml::Node &cfg, double dt)
    : pos_error_integrator_max_(kDefaultPosIntegratorMax),
      q_error_integrator_max_(kDefaultQIntegratorMax),
      pos_error_integrator_(Eigen::Vector3d::Zero()),
      q_error_integrator_(Eigen::Vector3d::Zero()),
      p_BO_(Eigen::Vector3d::Zero()), q_BO_(Eigen::Quaterniond::Identity()),
      filter_cutoff_f_(kDefaultFilterCutoffFrequency), arm_fixed_(false),
      fixed_arm_z_(kDefaultFixedArmZ), filter_base_wrench_(false), cfg_(cfg) {
  params_ = std::make_shared<delta_control::DeltaControllerParameters>();
  state_ = std::make_shared<delta_control::DeltaState>();
  reference_ = std::make_shared<delta_control::DeltaRef>();
  kin_ = std::make_shared<delta_control::Kinematics>(params_);
  dyn_ = std::make_shared<delta_control::Dynamics>(params_);

  sample_time_ = dt;
  prev_q_cmd_.setZero();
  for (int i = 0; i < 3; i++) {
    joints_.push_back(new raisim_actuators::delta_joint(sample_time_));
  }

  setMode(0);
  // Read Delta parameters.
  double r_B = cfg_["r_B"].template As<float>();
  double r_T = cfg_["r_T"].template As<float>();
  double l_P = cfg_["l_P"].template As<float>();
  double l_D = cfg_["l_D"].template As<float>();
  double theta_min = cfg_["theta_min"].template As<float>();
  double theta_max = cfg_["theta_max"].template As<float>();
  initialize(r_B, r_T, l_P, l_D, theta_min, theta_max, sample_time_);

  Eigen::Vector3d I_Base, I_O, I_A, I_P, pCom_Base, pCom_O, pCom_A, pCom_P,
      p_BO;
  Eigen::Quaterniond q_BO;
  double m_Base = cfg_["m_Base"].template As<float>();
  double m_O = cfg_["m_O"].template As<float>();
  double m_A = cfg_["m_A"].template As<float>();
  double m_P = cfg_["m_P"].template As<float>();

  // ROS_WARN_STREAM("Total delta mass: "<<(m_Base+m_O+m_A+m_P));
  I_Base = getParamVector3("I_Base");
  I_O = getParamVector3("I_O");
  I_A = getParamVector3("I_A");
  I_P = getParamVector3("I_P");
  pCom_Base = getParamVector3("pCom_Base");
  pCom_O = getParamVector3("pCom_O");
  pCom_A = getParamVector3("pCom_A");
  pCom_P = getParamVector3("pCom_P");
  p_BO = getParamVector3("p_BO");
  q_BO = getParamQuat("q_BO");
  initializeDynamics(m_Base, m_O, m_A, m_P, I_Base, I_O, I_A, I_P, pCom_Base,
                     pCom_O, pCom_A, pCom_P, p_BO, q_BO);

  joints_command_.Zero();
  joints_command_(0) = joints_[0]->getPos();
  joints_command_(1) = joints_[1]->getPos();
  joints_command_(2) = joints_[2]->getPos();
};

void DeltaController::sendActuatorsCommand(const Eigen::Vector3d jointsPos) {
  joints_[0]->send_cmd(jointsPos(0));
  joints_[1]->send_cmd(jointsPos(1));
  joints_[2]->send_cmd(jointsPos(2));
}

void DeltaController::sendActuatorsCommand() {
  joints_[0]->send_cmd(joints_command_(0));
  joints_[1]->send_cmd(joints_command_(1));
  joints_[2]->send_cmd(joints_command_(2));
}

void DeltaController::setJointAngles(const Eigen::Vector3d jointAngles) {
  joints_[0]->set_angle(jointAngles(0));
  joints_[1]->set_angle(jointAngles(1));
  joints_[2]->set_angle(jointAngles(2));
}

void DeltaController::initialize(const double &r_B, const double &r_T,
                                 const double &l_P, const double &l_D,
                                 const double &theta_min,
                                 const double &theta_max, const double &dt) {
  params_->updateParameters(r_B, r_T, l_P, l_D, theta_min, theta_max);
  if (dt > 0.0) {
    state_->setupButterworthFilters(1.0 / dt, filter_cutoff_f_);
  }
  for (int i = 0; i < 3; i++) {
    base_force_butterworth_[i].setup(1.0 / dt, filter_cutoff_f_);
    base_torque_butterworth_[i].setup(1.0 / dt, filter_cutoff_f_);
  }
  base_wrench_butterworth_initialized_ = true;
}

void DeltaController::initializeDynamics(
    const double &_m_Base, const double &_m_O, const double &_m_A,
    const double &_m_P, const Eigen::Vector3d &_I_Base,
    const Eigen::Vector3d &_I_O, const Eigen::Vector3d &_I_A,
    const Eigen::Vector3d &_I_P, const Eigen::Vector3d &_pCom_Base,
    const Eigen::Vector3d &_pCom_O, const Eigen::Vector3d &_pCom_A,
    const Eigen::Vector3d &_pCom_P, const Eigen::Vector3d &_p_BO,
    const Eigen::Quaterniond &_q_BO) {
  dyn_->updateDynamicParameters(_m_Base, _m_O, _m_A, _m_P, _I_Base, _I_O, _I_A,
                                _I_P, _pCom_Base, _pCom_O, _pCom_A, _pCom_P);
  // Since odometry is already in delta base frame, just use the offset to
  // transform forces before publishing. dyn_->updateBaseOffset(_p_BO, _q_BO);
  p_BO_ = _p_BO;
  q_BO_ = _q_BO;
}

void DeltaController::setGains(const double &p_gain, const double &i_gain,
                               const double &d_gain, const double &ff_gain) {
  params_->setGains(p_gain, i_gain, d_gain, ff_gain);
}

void DeltaController::setMasses(
    const double &mass_base, const double &mass_prox, const double &mass_dist,
    const double &mass_tool, const double &mass_elbow, const double &mass_motor,
    const double &radius_motor) {
  params_->setMasses(mass_base, mass_prox, mass_dist, mass_tool, mass_elbow,
                     mass_motor, radius_motor);
}

void DeltaController::updateErrorLimits(const double &max_pos_error,
                                        const double &max_vel_error) {
  params_->updateErrorLimits(max_pos_error, max_vel_error);
}

void DeltaController::setMode(int ctrl_mode) {
  delta_control::DeltaControlMode current_mode;
  params_->getControlMode(&current_mode);
  if (current_mode != delta_control::DeltaControlMode(ctrl_mode)) {
    params_->setControlMode(delta_control::DeltaControlMode(ctrl_mode));
    resetIntegrators();
  }
}

int DeltaController::getMode() const {
  delta_control::DeltaControlMode mode;
  params_->getControlMode(&mode);
  return int(mode);
}

void DeltaController::updateIntegrators(const Eigen::Vector3d &pos_error,
                                        const Eigen::Vector3d &q_error,
                                        const double &dt) {
  pos_error_integrator_ += pos_error * dt;
  if (pos_error_integrator_.norm() > pos_error_integrator_max_) {
    pos_error_integrator_ =
        pos_error_integrator_ *
        (pos_error_integrator_max_ / pos_error_integrator_.norm());
  }

  q_error_integrator_ += q_error * dt;
  if (q_error_integrator_.norm() > q_error_integrator_max_) {
    q_error_integrator_ = q_error_integrator_ * (q_error_integrator_max_ /
                                                 q_error_integrator_.norm());
  }
}

void DeltaController::resetIntegrators() {
  pos_error_integrator_.setZero();
  q_error_integrator_.setZero();
}

void DeltaController::updateState(const Eigen::Quaterniond &base_q_WB,
                                  const Eigen::Vector3d &base_vel_B,
                                  const Eigen::Vector3d &base_angvel_B,
                                  const Eigen::VectorXd &q_in,
                                  const Eigen::VectorXd &qd_in,
                                  const Eigen::VectorXd &effort_in,
                                  const double &dt) {
  Eigen::Vector3d q3_in, qd3_in, effort3_in;
  for (size_t i = 0; i < 3; ++i) {
    q3_in(i) = q_in(i);
    qd3_in(i) = qd_in(i);
    effort3_in(i) = effort_in(i);
  }
  updateState(base_q_WB, base_vel_B, base_angvel_B, q3_in, qd3_in, effort3_in,
              dt);
}

void DeltaController::updateState(const Eigen::Quaterniond &base_q_WB,
                                  const Eigen::Vector3d &base_vel_B,
                                  const Eigen::Vector3d &base_angvel_B,
                                  const Eigen::Vector3d &q_in,
                                  const Eigen::Vector3d &qd_in,
                                  const Eigen::Vector3d &effort_in,
                                  const double &dt) {
  Eigen::Vector3d tool_pos_B, tool_vel_B, q, qd, effort;
  Eigen::Matrix3d jac_inv(Eigen::Matrix3d::Zero());

  q = std::isnan(q_in.norm()) ? Eigen::Vector3d::Zero() : q_in;
  qd = std::isnan(qd_in.norm()) ? Eigen::Vector3d::Zero() : qd_in;
  effort = std::isnan(effort_in.norm()) ? Eigen::Vector3d::Zero()
                                        : Eigen::Vector3d(effort_in);

  fwkinPosition(&tool_pos_B, q);
  Eigen::Matrix3d J(Eigen::Matrix3d::Zero());
  kin_->jac(&J, state_->q(), state_->tool_pos_B());
  tool_vel_B = J * qd;

  updateState(base_q_WB, base_vel_B, base_angvel_B, q, qd, effort, tool_pos_B,
              tool_vel_B, dt);
}

Eigen::Vector3d DeltaController::getqPos() {
  Eigen::Vector3d q(joints_[0]->getPos(), joints_[1]->getPos(),
                    joints_[2]->getPos());
  return q;
}
Eigen::Vector3d DeltaController::getqVel() {
  Eigen::Vector3d qd(joints_[0]->getVel(), joints_[1]->getVel(),
                     joints_[2]->getVel());
  return qd;
}
Eigen::Vector3d DeltaController::getqAcc() {
  Eigen::Vector3d qdd(joints_[0]->getAcc(), joints_[1]->getAcc(),
                      joints_[2]->getAcc());
  return qdd;
}

bool DeltaController::fwkinVel(Eigen::Vector3d *ee_vel,
                               const Eigen::Vector3d &ee_pos) {
  Eigen::Matrix3d J(Eigen::Matrix3d::Zero());
  kin_->jac(&J, getqPos(), ee_pos);
  *ee_vel = J * getqVel();
  return true;
}

// void DeltaController::updateState(const Eigen::Quaterniond& base_q_WB,
//                                   const Eigen::Vector3d& base_vel_B,
//                                   const Eigen::Vector3d& base_angvel_B, const
//                                   double& dt) {
//   Eigen::Vector3d tool_pos_B, tool_vel_B, q, qd, effort;
//   Eigen::Matrix3d jac_inv(Eigen::Matrix3d::Zero());

//   Eigen::Vector3d q_in, qd_in;

//   q = std::isnan(q_in.norm()) ? Eigen::Vector3d::Zero() : q_in;
//   qd = std::isnan(qd_in.norm()) ? Eigen::Vector3d::Zero() : qd_in;
//   effort = std::isnan(effort_in.norm()) ? Eigen::Vector3d::Zero() :
//   Eigen::Vector3d(effort_in);

//   fwkinPosition(&tool_pos_B, q);
//   Eigen::Matrix3d J(Eigen::Matrix3d::Zero());
//   kin_->jac(&J, state_->q(), state_->tool_pos_B());
//   tool_vel_B = J * qd;

//   updateState(base_q_WB, base_vel_B, base_angvel_B, q, qd, effort,
//   tool_pos_B, tool_vel_B, dt);
// }

void DeltaController::updateState(
    const Eigen::Quaterniond &base_q_WB, const Eigen::Vector3d &base_vel_B,
    const Eigen::Vector3d &base_angvel_B, const Eigen::Vector3d &q,
    const Eigen::Vector3d &qd, const Eigen::Vector3d &effort,
    const Eigen::Vector3d &tool_pos_B, const Eigen::Vector3d &tool_vel_B,
    const double &dt) {
  state_->update(q, qd, effort, dt);
  state_->updateBase(base_q_WB, base_vel_B, base_angvel_B, dt);
  state_->updateTool(tool_pos_B, tool_vel_B, dt);
}

void DeltaController::updateReference(const Eigen::Vector3d &pos_B,
                                      const Eigen::Vector3d &vel_B,
                                      const Eigen::Vector3d &acc_B,
                                      const Eigen::Vector3d &force_B) {
  reference_->update(pos_B, vel_B, acc_B, force_B);
}

void DeltaController::getWorkspacePositionError(Eigen::Vector3d *tool_pos_err) {
  assert(tool_pos_err);
  *tool_pos_err = state_->tool_pos_B() - reference_->pos();
}

void DeltaController::getToolPosition(Eigen::Vector3d *tool_pos) {
  assert(tool_pos);
  *tool_pos = state_->tool_pos_B();
}

void DeltaController::getJointspacePositionError(Eigen::Vector3d *q_err) {
  Eigen::Vector3d q_ref;
  inkinPosition(&q_ref, reference_->pos());
  *q_err = state_->q() - q_ref;
}

void DeltaController::getControlCommand(Eigen::VectorXd *delta_pos_cmd,
                                        Eigen::VectorXd *delta_vel_cmd,
                                        Eigen::VectorXd *delta_eff_cmd,
                                        const double &dt) {
  assert(delta_pos_cmd);
  assert(delta_vel_cmd);
  assert(delta_eff_cmd);
  delta_pos_cmd->resize(3);
  delta_vel_cmd->resize(3);
  delta_eff_cmd->resize(3);
  delta_pos_cmd->setZero();
  delta_vel_cmd->setZero();
  delta_eff_cmd->setZero();

  Eigen::Vector3d ee_pos_ref, ee_vel_ref, ee_acc_ref;
  if (arm_fixed_) {
    ee_pos_ref << 0.0, 0.0, fixed_arm_z_;
    ee_vel_ref.setZero();
    ee_acc_ref.setZero();
  } else {
    // Zero velocity, accel out of ws boundary.
    ee_pos_ref = reference_->pos();
    ee_vel_ref = reference_->vel();
    ee_acc_ref = reference_->acc();
    if (projectToWorkspace(&ee_pos_ref, reference_->pos())) {
      ee_vel_ref.setZero();
      ee_acc_ref.setZero();
    }
  }

  // Always compute joint position command
  Eigen::Vector3d delta_pos3_cmd;
  bool success = inkinPosition(&delta_pos3_cmd, ee_pos_ref);
  if (success) {
    *delta_pos_cmd = Eigen::VectorXd(delta_pos3_cmd);
  } else {
    *delta_pos_cmd = Eigen::VectorXd(prev_q_cmd_);
  }

  // Compute error and update integrators
  Eigen::Vector3d ee_pos_error = ee_pos_ref - state_->tool_pos_B();
  ee_pos_error = limitVector(ee_pos_error, params_->max_pos_error());

  Eigen::Vector3d ee_vel_error = ee_vel_ref - state_->tool_vel_B();
  ee_vel_error = limitVector(ee_vel_error, params_->max_vel_error());

  Eigen::Vector3d q_error = *delta_pos_cmd - state_->q();
  updateIntegrators(ee_pos_error, q_error, dt);

  switch (params_->control_mode()) {
  case delta_control::DeltaControlMode::Position: {
    // Apply limited position error.
    success =
        inkinPosition(&delta_pos3_cmd, state_->tool_pos_B() + ee_pos_error);
    if (success) {
      *delta_pos_cmd = Eigen::VectorXd(delta_pos3_cmd);
    } else {
      *delta_pos_cmd = Eigen::VectorXd(prev_q_cmd_);
    }
  }
  case delta_control::DeltaControlMode::Velocity: { // Workspace velocity PID.
    Eigen::Vector3d ee_vel_cmd;
    ee_vel_cmd = params_->p_gain() * ee_pos_error +
                 params_->i_gain() * pos_error_integrator_ +
                 params_->d_gain() * ee_vel_error +
                 params_->ff_gain() * ee_vel_ref;
    Eigen::Vector3d delta_vel3_cmd;
    inkinVelocity(&delta_vel3_cmd, ee_vel_cmd);
    *delta_vel_cmd = delta_vel3_cmd;
    break;
  }
  }
  prev_q_cmd_ = *delta_pos_cmd;
} // namespace delta_control

Eigen::Vector3d
DeltaController::getParamVector3(const std::string &param_name) {
  double x = cfg_[param_name]["x"].template As<float>();
  double y = cfg_[param_name]["y"].template As<float>();
  double z = cfg_[param_name]["z"].template As<float>();
  return Eigen::Vector3d(x, y, z);
}

Eigen::Quaterniond
DeltaController::getParamQuat(const std::string &param_name) {
  double w = cfg_[param_name]["w"].template As<float>();
  double x = cfg_[param_name]["x"].template As<float>();
  double y = cfg_[param_name]["y"].template As<float>();
  double z = cfg_[param_name]["z"].template As<float>();
  return Eigen::Quaterniond(w, x, y, z);
}

void DeltaController::getTargetPos(Eigen::Vector3d *delta_pos_ref) {
  assert(delta_pos_ref);
  delta_pos_ref->setZero();
  bool projected = projectToWorkspace(delta_pos_ref, reference_->pos());
}

bool DeltaController::fwkinPosition(Eigen::Vector3d *ee_pos) {
  return fwkinPosition(ee_pos, getqPos());
}

bool DeltaController::fwkinPosition(Eigen::Vector3d *ee_pos,
                                    const Eigen::Vector3d &q) {
  assert(ee_pos);
  // The forward kinematics of the Clavel's Delta are defined by the
  // intersection of 3 spheres located at at a horizontal offset from the knee
  // joint (offset is the difference in platform radii) as described in
  // Carp-Ciocardia, D. C. "Dynamic analysis of Clavel's delta parallel robot."
  // 2003
  //
  // Implementation from https://github.com/vvhitedog/three_sphere_intersection/
  //  std::cout << "delta controller q: " << q << std::endl;
  double R_s = params_->r_B() - params_->r_T();
  // Sphere centers defined by knees plus offset;
  std::vector<Eigen::Vector3d> spheres(3);
  double r_sphere = params_->l_D();

  for (int i = 0; i < spheres.size(); ++i) {
    double cos_theta_L = std::cos(q[i]) * params_->l_P();
    spheres[i] << std::cos(params_->gamma()(i)) * (R_s + cos_theta_L),
        std::sin(params_->gamma()(i)) * (R_s + cos_theta_L),
        std::sin(q[i]) * params_->l_P();
  }

  // Find circle inscribed by intersection of two spheres.
  Eigen::Vector3d n = (spheres[1] - spheres[0]).normalized();
  double d = (spheres[1] - spheres[0]).norm();
  // The following center point is only valid for spheres of equal size.
  Eigen::Vector3d c_circ = spheres[0] + 0.5 * n * d;
  double r_circ = std::sqrt(r_sphere * r_sphere - 0.25 * d * d);

  // Find 2 vectors u,v spanning plane of circle.
  Eigen::Vector3d u, v;
  u = Eigen::Vector3d::UnitX();
  if (std::abs(u.dot(n)) < 1e-6 || std::abs(u.dot(n)) - M_PI < 1e-6) {
    u = Eigen::Vector3d::UnitY();
  }
  u = u - u.dot(n) * n;
  u.normalize();
  v = n.cross(u);

  // Find intersections of third sphere with circle.
  Eigen::Vector3d cd = c_circ - spheres[2];
  double gamma = r_sphere * r_sphere - cd.dot(cd) - r_circ * r_circ;
  double alpha = 2.0 * cd.dot(u) * r_circ;
  double beta = 2.0 * cd.dot(v) * r_circ;

  double var_c = gamma - alpha;
  double var_b = -2.0 * beta;
  double var_a = gamma + alpha;
  double discriminant = var_b * var_b - 4.0 * var_a * var_c;
  if (discriminant < 0.0)
    return false;

  double sq_discriminant = std::sqrt(discriminant);
  double t0 = (-var_b + sq_discriminant) / (2.0 * var_a);
  double t1 = (-var_b - sq_discriminant) / (2.0 * var_a);
  t0 = 2.0 * std::atan(t0);
  t1 = 2.0 * std::atan(t1);

  t0 = t0 > 0 ? t0 : t0 + 2.0 * M_PI;
  t1 = t1 > 0 ? t1 : t1 + 2.0 * M_PI;

  Eigen::Vector3d point0 =
      c_circ + r_circ * (std::cos(t0) * u + std::sin(t0) * v);
  Eigen::Vector3d point1 =
      c_circ + r_circ * (std::cos(t1) * u + std::sin(t1) * v);

  // Select non-inverted solution (farthest from delta base).
  *ee_pos = point0[2] > point1[2] ? point0 : point1;

  return true;
}

void DeltaController::fixArm(const bool set_fixed) { arm_fixed_ = set_fixed; }

void DeltaController::setFixedZ(const double &fixed_arm_z) {
  fixed_arm_z_ = fixed_arm_z;
}

void DeltaController::setBaseWrenchFilter(const bool set_filter,
                                          const double &cutoff_freq,
                                          const double &dt) {
  filter_base_wrench_ = set_filter;
  filter_cutoff_f_ = cutoff_freq;
  for (int i = 0; i < 3; i++) {
    base_force_butterworth_[i].setup(1.0 / dt, filter_cutoff_f_);
    base_torque_butterworth_[i].setup(1.0 / dt, filter_cutoff_f_);
  }
}

bool DeltaController::inkinPosition(Eigen::Vector3d *q,
                                    const Eigen::Vector3d &ee_pos) {
  assert(q);
  // The inverse kinematics computation is based on:
  // "The Delta Parallel Robot: Kinematics Solutions"
  // by Robert L. Williams II, Ph.D., williar4@ohio.edu

  Eigen::Vector3d theta; // Joint angles [rad]
  theta.setZero();
  double r_s = params_->r_B() - params_->r_T();

  // Evaluate joint solutions for each leg.
  for (int i = 0; i < 3; i++) {
    double G = ee_pos(0) * ee_pos(0) + ee_pos(1) * ee_pos(1) +
               ee_pos(2) * ee_pos(2) + r_s * r_s +
               params_->l_P() * params_->l_P() -
               params_->l_D() * params_->l_D() -
               2.0 * r_s *
                   (ee_pos(0) * std::cos(params_->gamma()(i)) +
                    ee_pos(1) * std::sin(params_->gamma()(i)));
    double E = 2.0 * params_->l_P() *
               (r_s - ee_pos(0) * std::cos(params_->gamma()(i)) -
                ee_pos(1) * std::sin(params_->gamma()(i)));
    double F = 2.0 * ee_pos(2) * params_->l_P();
    double tmp = std::sqrt(E * E + F * F - G * G);

    // Two resulting solutions for each leg: knee in and knee out;
    double sol_1 = 2.0 * std::atan2((F + tmp), (G - E));
    double sol_2 = 2.0 * std::atan2((F - tmp), (G - E));

    // wrap angles to min servo offset
    sol_1 = angleWrap(sol_1, params_->theta_min());
    sol_2 = angleWrap(sol_2, params_->theta_min());

    // To be within the workspace, the solution must be:
    //  - real
    //  - within joint limits
    //  - traversable from zero position without inversion (always knee out)
    bool sol_1_feasible =
        !std::isnan(sol_1) &&
        (sol_1 >= params_->theta_min() && sol_1 <= params_->theta_max());
    bool sol_2_feasible =
        !std::isnan(sol_2) &&
        (sol_2 >= params_->theta_min() && sol_2 <= params_->theta_max());

    if (sol_1_feasible && sol_2_feasible) {
      // Both solutions are feasible. Choose smaller angle (knee outward).
      theta(i) = (sol_1 < sol_2) ? sol_1 : sol_2;
    } else if (sol_1_feasible) {
      // Only solution 1 is feasible.
      theta(i) = sol_1;
    } else if (sol_2_feasible) {
      // Only solution 2 is feasible.
      theta(i) = sol_2;
    } else {
      // No feasible solutions
      *q = Eigen::Vector3d::Zero();
      return false;
    }
  }
  *q = theta;
  return true;
}

bool DeltaController::inkinVelocity(Eigen::Vector3d *qdot,
                                    const Eigen::Vector3d &ee_vel) {
  assert(qdot);
  // The equations in matrix form are: M*Pdot = V*qdot
  // Performing inversion of V (which is a diagonal 3x3 matrix):
  // qdot = J^-1 * Pdot, where J^-1 = V^-1 * M is the inverse Jacobian.

  Eigen::Matrix3d J_inv(Eigen::Matrix3d::Zero());
  kin_->jacInv(&J_inv, state_->q(), state_->tool_pos_B());

  // Compute qdot.
  *qdot = J_inv * ee_vel;
  if (std::isnan(qdot->norm())) {
    qdot->setZero();
    return false;
  }
  return true;
}

void DeltaController::getBaseWrench(Eigen::Vector3d *force_B,
                                    Eigen::Vector3d *torque_B,
                                    const Eigen::Quaterniond &q_WB,
                                    const Eigen::Vector3d &B_om_WB /*angvel*/,
                                    const Eigen::Vector3d &B_dom_WB /*angacc*/,
                                    const Eigen::Vector3d &B_dv_WB /*linacc*/,
                                    const Eigen::Vector3d &p_OE /*tool_pos*/,
                                    const Eigen::Vector3d &v_OE /*tool_vel*/,
                                    const Eigen::Vector3d &dv_OE /*tool_acc*/) {
  assert(force_B);
  assert(torque_B);

  dyn_->computeForces(getqPos(), getqVel(), getqAcc(), q_WB, B_om_WB, B_dom_WB,
                      B_dv_WB, p_OE, v_OE, dv_OE);

  Eigen::Vector3d force_O_raw, torque_O_raw, force_B_raw, torque_B_raw;
  dyn_->getBaseWrench(&force_O_raw, &torque_O_raw);

  // Convert from delta base to floating base frame (negate for reaction force)
  Eigen::Matrix3d R_BO = q_BO_.toRotationMatrix();
  torque_B_raw = R_BO * torque_O_raw;
  force_B_raw = R_BO * force_O_raw + p_BO_.cross(torque_B_raw);

  // if (filter_base_wrench_) {
  //   if (base_wrench_butterworth_initialized_ && !force_B_raw.hasNaN() &&
  //   !torque_B_raw.hasNaN()) {
  //     Eigen::Vector3d force_B_filtered, torque_B_filtered;
  //     for (int i = 0; i < 3; i++) {
  //       force_B_filtered(i) =
  //       base_force_butterworth_[i].filter(force_B_raw(i));
  //       torque_B_filtered(i) =
  //       base_torque_butterworth_[i].filter(torque_B_raw(i));
  //     }
  //     past_base_force_.push_back(force_B_filtered);
  //     past_base_torque_.push_back(torque_B_filtered);
  //     *force_B = force_B_filtered;
  //     *torque_B = torque_B_filtered;
  //   } else {
  //     force_B->setZero();
  //     torque_B->setZero();
  //   }
  // } else {
  *force_B = force_B_raw;
  *torque_B = torque_B_raw;
  // }
}

void DeltaController::getWorkspaceRegion(int *upper_index, int *lower_index,
                                         const Eigen::Vector3d &point) {
  // Identify x/y region for upper and lower bounds.
  std::uint_fast8_t region = 0;
  if (point(1) > 0.0)
    region |= 0b00000001;
  if (point(1) > -std::sqrt(3.0) * point(0))
    region |= 0b00000010;
  if (point(1) > std::sqrt(3.0) * point(0))
    region |= 0b00000100;

  *upper_index = 0;
  *lower_index = 0;
  for (uint i = 0; i < 3; ++i) {
    *upper_index += !((region & mask[i]) ^ upper_region[i]) * i;
    *lower_index += !((region & mask[i]) ^ lower_region[i]) * i;
  }
}

bool DeltaController::projectToWorkspace(Eigen::Vector3d *projection,
                                         const Eigen::Vector3d &point) {
  // Projects to known workspace based on delta parameters.
  // Reuturn True if point is projected, False if already within workspace.
  assert(projection);
  Eigen::Vector3d projected_point(Eigen::Vector3d::Zero());
  bool point_modified = false;

  int upper_index, lower_index;
  getWorkspaceRegion(&upper_index, &lower_index, point);

  // Project down to maximum height
  point_modified |=
      projectToHeight(&projected_point, projected_point, params_->max_z(), -1);

  // Project onto upper bound sphere
  point_modified |=
      projectToSphere(&projected_point, point, params_->ub_sphere[upper_index],
                      params_->l_D() - params_->ws_buffer(), -1);
  // Project up to minimum height
  point_modified |=
      projectToHeight(&projected_point, projected_point, params_->min_z(), 1);

  // Project onto lower bound sphere
  point_modified |= projectToSphere(&projected_point, projected_point,
                                    params_->lb_sphere[lower_index],
                                    params_->l_D() + params_->ws_buffer(), 1);

  if (point_modified) {
    // Confirm projection is in same region.
    int up, lo;
    getWorkspaceRegion(&up, &lo, projected_point);
    if (up != upper_index || lo != lower_index)
      projectToWorkspace(&projected_point, projected_point);
  }

  *projection = projected_point;
  return point_modified;
}

bool DeltaController::projectToSphere(Eigen::Vector3d *projection,
                                      const Eigen::Vector3d &point,
                                      const Eigen::Vector3d &center,
                                      const double &radius,
                                      const int direction) {
  assert(projection);
  *projection = point;
  Eigen::Vector3d vec = point - center;
  if (direction * vec.norm() < direction * params_->l_D()) {
    *projection = center + vec * (params_->l_D() / vec.norm());
    return true;
  }
  return false;
}

bool DeltaController::projectToHeight(Eigen::Vector3d *projection,
                                      const Eigen::Vector3d &point,
                                      const double &height,
                                      const int direction) {
  assert(projection);
  *projection = point;
  if (direction * point(2) < direction * height) {
    (*projection)(2) = height;
    return true;
  }
  return false;
}

double DeltaController::angleWrap(const double &angle, const double &offset) {
  double x = std::fmod(angle - offset, 2.0 * M_PI);
  if (x < 0) {
    x += 2.0 * M_PI;
  }
  return x + offset;
}

Eigen::Vector3d DeltaController::limitVector(Eigen::Vector3d &vec,
                                             const double &limit) {
  Eigen::Vector3d limited_vec = vec;
  double norm = vec.norm();
  if (limit > 1e-8 && norm > limit) {
    limited_vec = vec * limit / norm;
  }
  return limited_vec;
}

Eigen::Matrix3d DeltaController::RotZ(const double &angle) {
  Eigen::Matrix3d R;
  R << std::cos(angle), -std::sin(angle), 0.0, std::sin(angle), std::cos(angle),
      0.0, 0.0, 0.0, 1.0;
  return R;
}

} // namespace delta_dynamics