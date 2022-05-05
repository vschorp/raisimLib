/*
 * Copyright 2021 Karen Bodie, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or
 * otherwise.
 *
 */

#ifndef DELTA_CONTROL_KINEMATICS_H
#define DELTA_CONTROL_KINEMATICS_H

#include <Eigen/Core>
#include "controller_helpers.h"

namespace delta_control {

class Kinematics {
 public:
  Kinematics(std::shared_ptr<DeltaControllerParameters> params)
      : params_(params){};
  ~Kinematics(){};

  void A_mat(Eigen::Matrix3d* A, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee) {
    assert(A);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "A.h"
    *A = A0;
  }

  void B_mat(Eigen::Matrix3d* B, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee) {
    assert(B);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "B.h"
    *B = A0;
  }

  void dAdq_mat(Eigen::Matrix3d* dAdq, const Eigen::Vector3d& q, const Eigen::Vector3d& qd,
                const Eigen::Vector3d& x_ee) {
    assert(dAdq);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "dAdq.h"
    *dAdq = A0;
  }

  void dBdq_mat(Eigen::Matrix3d* dBdq, const Eigen::Vector3d& q, const Eigen::Vector3d& qd,
                const Eigen::Vector3d& x_ee) {
    assert(dBdq);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "dBdq.h"
    *dBdq = A0;
  }

  void dAdx_mat(Eigen::Matrix3d* dAdx, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee,
                const Eigen::Vector3d& xd_ee) {
    assert(dAdx);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double xd = xd_ee(0);
    double yd = xd_ee(1);
    double zd = xd_ee(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "dAdx.h"
    *dAdx = A0;
  }

  void dBdx_mat(Eigen::Matrix3d* dBdx, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee,
                const Eigen::Vector3d& xd_ee) {
    assert(dBdx);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double xd = xd_ee(0);
    double yd = xd_ee(1);
    double zd = xd_ee(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "dBdx.h"
    *dBdx = A0;
  }

  void jacInv(Eigen::Matrix3d* J_inv, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee) {
    assert(J_inv);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "jac_inv.h"
    *J_inv = A0;
  }

  void jac(Eigen::Matrix3d* J, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee) {
    assert(J);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    Eigen::Matrix3d A0(Eigen::Matrix3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "jac.h"
    *J = A0;
  }

  void djacInv(Eigen::Vector3d* qdd, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee,
               const Eigen::Vector3d& qd, const Eigen::Vector3d& xd_ee,
               const Eigen::Vector3d& xdd_ee) {
    assert(qdd);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double xd = xd_ee(0);
    double yd = xd_ee(1);
    double zd = xd_ee(2);
    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);
    double xdd = xdd_ee(0);
    double ydd = xdd_ee(1);
    double zdd = xdd_ee(2);
    Eigen::Vector3d A0(Eigen::Vector3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "djac_inv.h"
    *qdd = A0;
  }

  void djac(Eigen::Vector3d* xdd_ee, const Eigen::Vector3d& q, const Eigen::Vector3d& x_ee,
            const Eigen::Vector3d& qd, const Eigen::Vector3d& xd_ee, const Eigen::Vector3d& qdd) {
    assert(xdd_ee);
    double x = x_ee(0);
    double y = x_ee(1);
    double z = x_ee(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double xd = xd_ee(0);
    double yd = xd_ee(1);
    double zd = xd_ee(2);
    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);
    double qdd1 = qdd(0);
    double qdd2 = qdd(1);
    double qdd3 = qdd(2);
    Eigen::Vector3d A0(Eigen::Vector3d::Zero());
    double r_b = params_->r_B();
    double r_p = params_->r_T();
    double l_p = params_->l_P();
    double l_d = params_->l_D();
#include "djac.h"
    *xdd_ee = A0;
  }

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::shared_ptr<DeltaControllerParameters> params_;
};

}  // namespace delta_control

#endif  // DELTA_CONTROL_KINEMATICS_H