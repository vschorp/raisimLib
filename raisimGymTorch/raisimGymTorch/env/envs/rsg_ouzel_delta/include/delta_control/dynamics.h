#ifndef DELTA_CONTROL_DYNAMICS_H
#define DELTA_CONTROL_DYNAMICS_H

#include "controller_helpers.h"
#include <Eigen/Eigen>

namespace delta_control {

class Dynamics {
  static constexpr double kGravity = 9.81;

  static constexpr double kDefaultMassBase = 0.53;
  static constexpr double kDefaultMassDeltaBase = 0.46;
  static constexpr double kDefaultMassPromixalLink = 0.06;
  static constexpr double kDefaultMassPlatform = 0.125;

  const Eigen::Vector3d kDefaultPosBaseToDelta =
      Eigen::Vector3d(0.0, 0.0, 0.065);
  const Eigen::Quaterniond kDefaultQBaseToDelta =
      Eigen::Quaterniond::Identity();

  const Eigen::Vector3d kDefaultIntertiaBase =
      1e-6 * Eigen::Vector3d(1581.5, 1269.7, 2825.0);
  const Eigen::Vector3d kDefaultIntertiaDeltaBase =
      1e-6 * Eigen::Vector3d(1696.4, 1696.4, 3059.1);
  const Eigen::Vector3d kDefaultIntertiaProximalLink =
      1e-6 * Eigen::Vector3d(22.0, 801.8, 817.0);
  const Eigen::Vector3d kDefaultIntertiaPlatform =
      1e-6 * Eigen::Vector3d(136.8, 143.9, 254.8);

  const Eigen::Vector3d kDefaultComOffsetBase =
      Eigen::Vector3d(0.0, 0.0, 0.003);
  const Eigen::Vector3d kDefaultComOffsetDeltaBase =
      Eigen::Vector3d(0.0, 0.0, -0.004);
  const Eigen::Vector3d kDefaultComOffsetProximalLink =
      Eigen::Vector3d(0.099, 0.0, 0.0);
  const Eigen::Vector3d kDefaultComOffsetPlatform =
      Eigen::Vector3d(0.0, 0.0, 0.0016);

public:
  Dynamics(std::shared_ptr<DeltaControllerParameters> params)
      : params_(params) {
    force_B_.setZero();
    torque_B_.setZero();
    torque_act_.setZero();
    force_ee_.setZero();
  };
  ~Dynamics(){};

  void updateDynamicParameters(
      const double &_m_Base, const double &_m_O, const double &_m_A,
      const double &_m_P, const Eigen::Vector3d &_I_Base,
      const Eigen::Vector3d &_I_O, const Eigen::Vector3d &_I_A,
      const Eigen::Vector3d &_I_P, const Eigen::Vector3d &_pCom_Base,
      const Eigen::Vector3d &_pCom_O, const Eigen::Vector3d &_pCom_A,
      const Eigen::Vector3d &_pCom_P) {
    m_Base = _m_Base;
    m_O = _m_O;
    m_A = _m_A;
    m_P = _m_P;
    Ixx_Base = _I_Base.x();
    Iyy_Base = _I_Base.y();
    Izz_Base = _I_Base.z();
    Ixx_O = _I_O.x();
    Iyy_O = _I_O.y();
    Izz_O = _I_O.z();
    Ixx_A = _I_A.x();
    Iyy_A = _I_A.y();
    Izz_A = _I_A.z();
    Ixx_P = _I_P.x();
    Iyy_P = _I_P.y();
    Izz_P = _I_P.z();
    pComx_Base = _pCom_Base.x();
    pComy_Base = _pCom_Base.y();
    pComz_Base = _pCom_Base.z();
    pComx_O = _pCom_O.x();
    pComy_O = _pCom_O.y();
    pComz_O = _pCom_O.z();
    pComx_A = _pCom_A.x();
    pComy_A = _pCom_A.y();
    pComz_A = _pCom_A.z();
    pComx_P = _pCom_P.x();
    pComy_P = _pCom_P.y();
    pComz_P = _pCom_P.z();
  }

  void updateBaseOffset(const Eigen::Vector3d &_p_BO,
                        const Eigen::Quaterniond &_q_BO) {
    x_BO = _p_BO.x();
    y_BO = _p_BO.y();
    z_BO = _p_BO.z();
    qw_BO = _q_BO.w();
    qx_BO = _q_BO.x();
    qy_BO = _q_BO.y();
    qz_BO = _q_BO.z();
  }

  void getBaseWrench(Eigen::Vector3d *force_B, Eigen::Vector3d *torque_B) {
    assert(force_B);
    assert(torque_B);
    *force_B = force_B_;
    *torque_B = torque_B_;
  }

  void getEndEffectorForce(Eigen::Vector3d *force_ee) {
    assert(force_ee);
    *force_ee = force_ee_;
  }

  void getActuatorTorques(Eigen::Vector3d *torque_act) {
    assert(torque_act);
    *torque_act = torque_act_;
  }

  void
  computeForces(const Eigen::Vector3d &q_a, const Eigen::Vector3d &qd_a,
                const Eigen::Vector3d &qdd_a, const Eigen::Quaterniond &q_WB,
                const Eigen::Vector3d &B_om_WB, const Eigen::Vector3d &B_dom_WB,
                const Eigen::Vector3d &B_dv_WB, const Eigen::Vector3d &p_OE,
                const Eigen::Vector3d &v_OE, const Eigen::Vector3d &dv_OE) {
    double q_a1 = q_a(0);
    double q_a2 = q_a(1);
    double q_a3 = q_a(2);

    double qd_a1 = qd_a(0);
    double qd_a2 = qd_a(1);
    double qd_a3 = qd_a(2);

    double qdd_a1 = qdd_a(0);
    double qdd_a2 = qdd_a(1);
    double qdd_a3 = qdd_a(2);

    double qw_WB = q_WB.w();
    double qx_WB = q_WB.x();
    double qy_WB = q_WB.y();
    double qz_WB = q_WB.z();

    double B_omx_WB = B_om_WB.x();
    double B_omy_WB = B_om_WB.y();
    double B_omz_WB = B_om_WB.z();

    double B_domx_WB = B_dom_WB.x();
    double B_domy_WB = B_dom_WB.y();
    double B_domz_WB = B_dom_WB.z();

    double B_dvx_WB = B_dv_WB.x();
    double B_dvy_WB = B_dv_WB.y();
    double B_dvz_WB = B_dv_WB.z();

    double x_OE = p_OE.x();
    double y_OE = p_OE.y();
    double z_OE = p_OE.z();

    double vx_OE = v_OE.x();
    double vy_OE = v_OE.y();
    double vz_OE = v_OE.z();

    double dvx_OE = dv_OE.x();
    double dvy_OE = dv_OE.y();
    double dvz_OE = dv_OE.z();

    // Update computed dynamics
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "tau_tree.h"
  }

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constant kinematic parameters
  std::shared_ptr<DeltaControllerParameters> params_;

  double x_BO = kDefaultPosBaseToDelta.x();
  double y_BO = kDefaultPosBaseToDelta.y();
  double z_BO = kDefaultPosBaseToDelta.z();
  double qw_BO = kDefaultQBaseToDelta.w();
  double qx_BO = kDefaultQBaseToDelta.x();
  double qy_BO = kDefaultQBaseToDelta.y();
  double qz_BO = kDefaultQBaseToDelta.z();

  // Constant dynamic parameters
  double g = kGravity;
  double m_Base = kDefaultMassBase;
  double m_O = kDefaultMassDeltaBase;
  double m_A = kDefaultMassPromixalLink;
  double m_P = kDefaultMassPlatform;
  double Ixx_Base = kDefaultIntertiaBase(0);
  double Iyy_Base = kDefaultIntertiaBase(1);
  double Izz_Base = kDefaultIntertiaBase(2);
  double Ixx_O = kDefaultIntertiaDeltaBase(0);
  double Iyy_O = kDefaultIntertiaDeltaBase(1);
  double Izz_O = kDefaultIntertiaDeltaBase(2);
  double Ixx_A = kDefaultIntertiaProximalLink(0);
  double Iyy_A = kDefaultIntertiaProximalLink(1);
  double Izz_A = kDefaultIntertiaProximalLink(2);
  double Ixx_P = kDefaultIntertiaPlatform(0);
  double Iyy_P = kDefaultIntertiaPlatform(1);
  double Izz_P = kDefaultIntertiaPlatform(2);
  double pComx_Base = kDefaultComOffsetBase(0);
  double pComy_Base = kDefaultComOffsetBase(1);
  double pComz_Base = kDefaultComOffsetBase(2);
  double pComx_O = kDefaultComOffsetDeltaBase(0);
  double pComy_O = kDefaultComOffsetDeltaBase(1);
  double pComz_O = kDefaultComOffsetDeltaBase(2);
  double pComx_A = kDefaultComOffsetProximalLink(0);
  double pComy_A = kDefaultComOffsetProximalLink(1);
  double pComz_A = kDefaultComOffsetProximalLink(2);
  double pComx_P = kDefaultComOffsetPlatform(0);
  double pComy_P = kDefaultComOffsetPlatform(1);
  double pComz_P = kDefaultComOffsetPlatform(2);

  // Computed dynamics
  Eigen::Vector3d force_B_;
  Eigen::Vector3d torque_B_;
  Eigen::Vector3d torque_act_;
  Eigen::Vector3d force_ee_;
};

} // namespace delta_control

#endif // DELTA_CONTROL_DYNAMICS_H