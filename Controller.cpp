/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"
#include <iostream>
#include <nlopt.hpp>
#include <string>

//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _LeftendEffector,
                       dart::dynamics::BodyNode* _RightendEffector)
    : mRobot(_robot),
      mLeftEndEffector(_LeftendEffector),
      mRightEndEffector(_RightendEffector) {
  assert(_robot != nullptr);
  assert(_LeftendEffector != nullptr);
  assert(_RightendEffector != nullptr);

  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;

  mKp.setZero();
  mKv.setZero();

  for (int i = 0; i < 3; ++i) {
    mKp(i, i) = 750.0;
    mKv(i, i) = 250.0;
  }

  // Remove position limits
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);

  // Set joint damping
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);

  mPOrR = Eigen::MatrixXd::Zero(3, dof);
  mPOrL = Eigen::MatrixXd::Zero(3, dof);
  mbOrL = Eigen::VectorXd::Zero(3);
  mbOrR = Eigen::VectorXd::Zero(3);
  mWOrR = 1;
  mWOrL = 1;
  mKpOr = 15;
  mKvOr = 0.2;

  mInverseKinematicsOnArms = false;
  mCOMAngleControl = false;
  mMaintainInitCOMDistance = true;
  mCOMPDControl = true;
  mInitCOMDistance = mRobot->getCOM().norm();
  std::cout << "mInitCOMDistance: " << mInitCOMDistance << std::endl;
  mKpCOM = 5.0;
  mKvCOM = 25.0;
  mWBal = Eigen::Matrix3d::Zero();
  int x0 = 2;
  int y0 = 0;
  int z0 = 1;
  mWBal(x0, x0) = 10, mWBal(y0, y0) = 0;
  mWBal(z0, z0) = 1;

  // PBal and bBal size based on mCOMAngleControl
  int numBodyLinks = mRobot->getNumDofs();
  if (mCOMAngleControl) {
    mPBal = Eigen::MatrixXd::Zero(1, numBodyLinks);
    mbBal = Eigen::VectorXd::Zero(1);
  } else {
    mPBal = Eigen::MatrixXd::Zero(3, numBodyLinks);
    mbBal = Eigen::VectorXd::Zero(3);
  }
  mSteps = 0;
  th_log = std::ofstream("/usr/local/share/krang/fixed_wheels/th_log");
  mWaistLocked = true;
  if (mWaistLocked)
    mRobot->getJoint("JWaist")->setActuatorType(
        dart::dynamics::Joint::ActuatorType::LOCKED);
  mOptDim = (mWaistLocked ? numBodyLinks - 1 : numBodyLinks);
  mddqBodyRef = Eigen::VectorXd::Zero(mOptDim);
  mMM = Eigen::MatrixXd::Zero(mOptDim, mOptDim);
  mhh = Eigen::VectorXd::Zero(mOptDim);
  mForces = Eigen::VectorXd::Zero(mOptDim);
}

//=========================================================================
Controller::~Controller() { th_log.close(); }
//=========================================================================
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

//=========================================================================
void printMatrix(Eigen::MatrixXd A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      std::cout << A(i, j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//========================================================================
double optFunc(const std::vector<double>& x, std::vector<double>& grad,
               void* my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams*>(my_func_data);
  size_t n = x.size();
  Eigen::VectorXd X = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; i++) X(i) = x[i];

  if (!grad.empty()) {
    Eigen::MatrixXd mGrad =
        optParams->P.transpose() * (optParams->P * X - optParams->b);
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5 * pow((optParams->P * X - optParams->b).norm(), 2));
}

//==============================================================================
std::vector<int> getChainDofIndices(dart::dynamics::BodyNode* body) {
  std::vector<int> v;
  auto p = body->getParentJoint();
  while (p->getNumDofs()) {
    v.insert(v.begin(), p->getDof(0)->getIndexInSkeleton());
    p = p->getParentBodyNode()->getParentJoint();
  }
  return v;
}
//==============================================================================
void Controller::setLeftOrientationOptParams(
    const Eigen::Vector3d& _LeftTargetRPY) {
  int numDof = mRobot->getNumDofs();
  int numTaskDof = 3;
  int numPassiveJoints = 0;
  int numLowerBodyLinksOnBase = 2;
  int numArmJoints = 7;

  static Eigen::Quaterniond quatRef, quat;
  static double quatRef_w, quat_w;
  static Eigen::Vector3d quatRef_xyz, quat_xyz, quatError_xyz, w, dwref, wref;
  static Eigen::MatrixXd JwL_small(
      numTaskDof, numPassiveJoints + numLowerBodyLinksOnBase + numArmJoints),
      dJwL_small(numTaskDof,
                 numPassiveJoints + numLowerBodyLinksOnBase + numArmJoints);
  static Eigen::MatrixXd JwL_full(numTaskDof, numDof),
      dJwL_full(numTaskDof, numDof);

  // Reference orientation (TargetRPY is assumed to be in Frame 0)
  quatRef = Eigen::Quaterniond(
      Eigen::AngleAxisd(_LeftTargetRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(_LeftTargetRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(_LeftTargetRPY(2), Eigen::Vector3d::UnitZ()));
  quatRef_w = quatRef.w();
  quatRef_xyz << quatRef.x(), quatRef.y(), quatRef.z();
  if (quatRef_w < 0) {
    quatRef_w *= -1.0;
    quatRef_xyz *= -1.0;
  }

  // Current orientation in Frame 0
  Eigen::Vector3d currentRPY =
      dart::math::matrixToEulerXYZ(mLeftEndEffector->getTransform().rotation());
  quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(currentRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(currentRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(currentRPY(2), Eigen::Vector3d::UnitZ()));
  // quat =
  // Eigen::Quaterniond(mRot0*mLeftEndEffector->getTransform().rotation());
  quat_w = quat.w();
  quat_xyz << quat.x(), quat.y(), quat.z();
  if (pow(-quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2) <
      pow(quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2)) {
    quat_w *= -1.0;
    quat_xyz *= -1.0;
  }

  // Orientation error
  quatError_xyz =
      quatRef_w * quat_xyz - quat_w * quatRef_xyz + quatRef_xyz.cross(quat_xyz);

  // Jacobian
  JwL_small = mLeftEndEffector->getAngularJacobian();
  JwL_full.setZero();
  Eigen::Vector3d zeroColumn(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 7> zero7Columns;
  zero7Columns << zeroColumn, zeroColumn, zeroColumn, zeroColumn, zeroColumn,
      zeroColumn, zeroColumn;
  JwL_full << JwL_small.block<3, 3>(0, 0), zeroColumn,
      JwL_small.block<3, 7>(0, 3), zero7Columns;

  // Jacobian Derivative
  dJwL_small = mLeftEndEffector->getAngularJacobianDeriv();
  dJwL_full.setZero();
  dJwL_full << dJwL_small.block<3, 3>(0, 0), zeroColumn,
      dJwL_small.block<3, 7>(0, 3), zero7Columns;

  // Current angular speed in frame 0 and Reference angular acceleration of
  // the end-effector in frame 0
  w = JwL_full * mdqBody;
  dwref = -mKpOr * quatError_xyz - mKvOr * w;

  // P and b
  mPOrL = mWOrL * JwL_full;
  mbOrL = -mWOrL * (dJwL_full * mdqBody - dwref);
}

//==============================================================================
void Controller::setRightOrientationOptParams(
    const Eigen::Vector3d& _RightTargetRPY) {
  int numDof = mRobot->getNumDofs();
  int numTaskDof = 3;
  int numPassiveJoints = 0;
  int numLowerBodyLinksOnBase = 2;
  int numArmJoints = 7;

  static Eigen::Quaterniond quatRef, quat;
  static double quatRef_w, quat_w;
  static Eigen::Vector3d quatRef_xyz, quat_xyz, quatError_xyz, w, dwref, wref;
  static Eigen::MatrixXd JwR_small(
      numTaskDof, numPassiveJoints + numLowerBodyLinksOnBase + numArmJoints),
      dJwR_small(numTaskDof,
                 numPassiveJoints + numLowerBodyLinksOnBase + numArmJoints);
  static Eigen::MatrixXd JwR_full(numTaskDof, numDof),
      dJwR_full(numTaskDof, numDof);

  // Reference orientation (TargetRPY is assumed to be in Frame 0)
  quatRef = Eigen::Quaterniond(
      Eigen::AngleAxisd(_RightTargetRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(_RightTargetRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(_RightTargetRPY(2), Eigen::Vector3d::UnitZ()));
  quatRef_w = quatRef.w();
  quatRef_xyz << quatRef.x(), quatRef.y(), quatRef.z();
  if (quatRef_w < 0) {
    quatRef_w *= -1.0;
    quatRef_xyz *= -1.0;
  }

  // Current orientation in Frame 0
  Eigen::Vector3d currentRPY = dart::math::matrixToEulerXYZ(
      mRightEndEffector->getTransform().rotation());
  quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(currentRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(currentRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(currentRPY(2), Eigen::Vector3d::UnitZ()));
  // quat =
  // Eigen::Quaterniond(mRot0*mRightEndEffector->getTransform().rotation());
  quat_w = quat.w();
  quat_xyz << quat.x(), quat.y(), quat.z();
  if (pow(-quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2) <
      pow(quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2)) {
    quat_w *= -1.0;
    quat_xyz *= -1.0;
  }

  // Orientation error
  quatError_xyz =
      quatRef_w * quat_xyz - quat_w * quatRef_xyz + quatRef_xyz.cross(quat_xyz);

  // Jacobian
  JwR_small = mRightEndEffector->getAngularJacobian();
  JwR_full.setZero();
  Eigen::Vector3d zeroColumn(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 7> zero7Columns;
  zero7Columns << zeroColumn, zeroColumn, zeroColumn, zeroColumn, zeroColumn,
      zeroColumn, zeroColumn;
  JwR_full << JwR_small.block<3, 3>(0, 0), zeroColumn, zero7Columns,
      JwR_small.block<3, 7>(0, 3);

  // Jacobian Derivative
  dJwR_small = mRightEndEffector->getAngularJacobianDeriv();
  dJwR_full.setZero();
  dJwR_full << dJwR_small.block<3, 3>(0, 0), zeroColumn, zero7Columns,
      dJwR_small.block<3, 7>(0, 3);

  // Current angular speed in frame 0 and Reference angular acceleration of
  // the end-effector in frame 0
  w = JwR_full * mdqBody;
  dwref = -mKpOr * quatError_xyz - mKvOr * w;

  // P and b
  mPOrR = mWOrR * JwR_full;
  mbOrR = -mWOrR * (dJwR_full * mdqBody - dwref);
}
//==============================================================================
/*
void Controller::setBalanceOptParams(double thref, double dthref,
                                     double ddthref) {
  int numDof = mRobot->getNumDofs();
  int numTaskDof = 3;
  int numBodyLinks = numDof;

  static Eigen::Vector3d COM, dCOM, COMref, dCOMref, ddCOMref, ddCOMStar,
      dCOMStar;
  static Eigen::MatrixXd JCOM_full(numTaskDof, numDof),
      dJCOM_full(numTaskDof, numDof);
  static Eigen::MatrixXd JCOM(numTaskDof, numBodyLinks),
      dJCOM(numTaskDof, numBodyLinks);
  static Eigen::VectorXd Jth(numBodyLinks), dJth(numBodyLinks);
  static Eigen::VectorXd thVec(numTaskDof), dthVec(numTaskDof);
  static double L, th, th_wrong, dth, ddthStar, dthStar;

  Eigen::Matrix3d mRot0 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mdRot0 = Eigen::Matrix3d::Zero();
  Eigen::Vector3d mxyz0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d mdxyz0 = Eigen::Vector3d::Zero();
  // COM coordinates in frame 0 represent which coordinate of getCOM()
  int x0 = 2;
  int y0 = 0;
  int z0 = 1;

  // x, dx, ddxStar
  COM = mRot0 * (mRobot->getCOM() - mxyz0);
  if (!mInverseKinematicsOnArms) {
    dCOM = mRot0 * (mRobot->getCOMLinearVelocity() - mdxyz0) +
           mdRot0 * (mRobot->getCOM() - mxyz0);
  }
  if (mCOMAngleControl) {
    th = atan2(COM(x0), COM(z0));
    if (!mInverseKinematicsOnArms) {
      dth = (cos(th) / COM(z0)) * (cos(th) * dCOM(x0) - sin(th) * dCOM(z0));
      ddthStar = ddthref - mKpCOM * (th - thref) - mKvCOM * (dth - dthref);
    } else {
      dthStar = dthref - mKpCOM * (th - thref);
    }
  } else {
    if (mMaintainInitCOMDistance)
      L = mInitCOMDistance;
    else
      L = pow(COM(x0) * COM(x0) + COM(z0) * COM(z0), 0.5);
    COMref(x0) = L * sin(thref); COMref(y0) = 0; COMref(z0) = L * cos(thref);
    dCOMref(x0) = (L * cos(thref) * dthref);
    dCOMref(y0) = 0.0;
    dCOMref(z0) = (-L * sin(thref) * dthref);
    if (!mInverseKinematicsOnArms) {
      ddCOMref(x0) = (-L * sin(thref) * dthref * dthref +
                   L * cos(thref) * ddthref);
      ddCOMref(y0) = 0.0;
      ddCOMref(z0) = (-L * cos(thref) * dthref * dthref - L * sin(thref) *
ddthref); ddCOMStar = ddCOMref - mKpCOM * (COM - COMref) - mKvCOM * (dCOM -
dCOMref); } else { dCOMStar = dCOMref - mKpCOM * (COM - COMref);
    }
  }

  // Jacobian
  JCOM = mRobot->getCOMLinearJacobian();
  if (mCOMAngleControl) {
    thVec << cos(th), 0.0, -sin(th);
    Jth = (cos(th) * thVec * JCOM) / COM(z0);
  }

  // Jacobian derivative
  if (!mInverseKinematicsOnArms) {
    dJCOM = mRobot->getCOMLinearJacobianDeriv();
    if (mCOMAngleControl) {
      dthVec << -sin(th), 0.0, -cos(th);
      dJth = (-sin(th) * thVec * JCOM * dth + cos(th) * dthVec * JCOM * dth +
              cos(th) * thVec * dJCOM - dCOM(z0) * Jth) /
             COM(z0);
    }

    // P and b
    if (mCOMAngleControl) {
      mPBal << mWBal(x0, x0) * Jth;
      mbBal << mWBal(x0, x0) * ((-dJth * mdqBody)(x0) +
                                (mCOMPDControl ? ddthStar : ddthref));
      // mbBal << mWBal(0, 0) *
      //             (-dJth * mdqBody + (mCOMPDControl ? ddthStar : ddthref));
    } else {
      mPBal << mWBal * JCOM;
      mbBal << mWBal *
                   (-dJCOM * mdqBody + (mCOMPDControl ? ddCOMStar : ddCOMref));
    }
  } else {
    // P and b
    if (mCOMAngleControl) {
      mPBal << mWBal(x0, x0) * Jth;
      mbBal << mWBal(x0, x0) * (mCOMPDControl ? dthStar : dthref);
    } else {
      mPBal << mWBal * JCOM;
      mbBal << mWBal * (mCOMPDControl ? dCOMStar : dCOMref);
    }
  }
  std::cout << std::endl << "mPBal: " << std::endl << mPBal << std::endl;
  std::cout << "mbBal: " << std::endl << mbBal << std::endl;
}
*/

void Controller::setBalanceOptParams(double thref, double dthref,
                                     double ddthref) {
  int numDof = mRobot->getNumDofs();
  int numTaskDof = 3;
  int numBodyLinks = numDof;

  static Eigen::Vector3d COM, dCOM, COMref, dCOMref, ddCOMref, ddCOMStar,
      dCOMStar;
  static Eigen::MatrixXd JCOM_full(numTaskDof, numDof),
      dJCOM_full(numTaskDof, numDof);
  static Eigen::MatrixXd JCOM(numTaskDof, numBodyLinks),
      dJCOM(numTaskDof, numBodyLinks);
  static Eigen::VectorXd Jth(numBodyLinks), dJth(numBodyLinks);
  static Eigen::VectorXd thVec(numTaskDof), dthVec(numTaskDof);
  static double L, th, th_wrong, dth, ddthStar, dthStar;

  Eigen::Matrix3d mRot0 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mdRot0 = Eigen::Matrix3d::Zero();
  Eigen::Vector3d mxyz0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d mdxyz0 = Eigen::Vector3d::Zero();
  // COM coordinates in frame 0 represent which coordinate of getCOM()
  int x0 = 2;
  int y0 = 0;
  int z0 = 1;

  // x, dx, ddxStar
  COM = mRot0 * (mRobot->getCOM() - mxyz0);
  dCOM = mRot0 * (mRobot->getCOMLinearVelocity() - mdxyz0) +
         mdRot0 * (mRobot->getCOM() - mxyz0);
  L = mInitCOMDistance;
  COMref(x0) = L * sin(thref);
  COMref(y0) = 0;
  COMref(z0) = L * cos(thref);
  dCOMref(x0) = (L * cos(thref) * dthref);
  dCOMref(y0) = 0.0;
  dCOMref(z0) = (-L * sin(thref) * dthref);
  ddCOMref(x0) = (-L * sin(thref) * dthref * dthref + L * cos(thref) * ddthref);
  ddCOMref(y0) = 0.0;
  ddCOMref(z0) = (-L * cos(thref) * dthref * dthref - L * sin(thref) * ddthref);
  ddCOMStar = ddCOMref - mKpCOM * (COM - COMref) - mKvCOM * (dCOM - dCOMref);

  // Jacobian
  JCOM = mRobot->getCOMLinearJacobian();
  // Jacobian derivative
  dJCOM = mRobot->getCOMLinearJacobianDeriv();

  // P and b
  mPBal << mWBal * JCOM;
  mbBal << mWBal * (-dJCOM * mdqBody + (mCOMPDControl ? ddCOMStar : ddCOMref));

  th = atan2(COM(x0), COM(z0));
  dth = (cos(th) / COM(z0)) * (cos(th) * dCOM(x0) - sin(th) * dCOM(z0));
  th_log << mSteps * 0.001 << " " << -thref << " " << -th << " " << -dthref
         << " " << -dth << std::endl;
}

//=========================================================================
void Controller::update(const Eigen::Vector3d& _LeftTargetPosition,
                        const Eigen::Vector3d& _RightTargetPosition,
                        const Eigen::Vector3d& _LeftTargetRPY,
                        const Eigen::Vector3d& _RightTargetRPY, double thref,
                        double dthref, double ddthref) {
  mSteps++;
  using namespace dart;
  using namespace std;
  const int dof = (const int)mRobot->getNumDofs();
  Eigen::VectorXd dq = mRobot->getVelocities();  // n x 1
  mdqBody = mRobot->getVelocities();             // n x 1
  double weightRight = 1.0, weightLeft = 1.0, weightRegulator = 1.0,
         weightBalance = 10.0;
  double KpzCOM = 750.0, KvzCOM = 250.0;

  // Left arm Position
  Eigen::Vector3d xLft = mLeftEndEffector->getTransform().translation();
  Eigen::Vector3d dxLft = mLeftEndEffector->getLinearVelocity();
  math::LinearJacobian JvLft = mLeftEndEffector->getLinearJacobian();  // 3 x n
  math::LinearJacobian dJvLft =
      mLeftEndEffector->getLinearJacobianDeriv();  // 3 x n
  Eigen::Vector3d ddxrefLft = -mKp * (xLft - _LeftTargetPosition) - mKv * dxLft;
  Eigen::Vector3d zeroColumn(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 7> zero7Columns;
  zero7Columns << zeroColumn, zeroColumn, zeroColumn, zeroColumn, zeroColumn,
      zeroColumn, zeroColumn;
  Eigen::Matrix<double, 3, 18> FullJacobianLeft;
  FullJacobianLeft << JvLft.block<3, 3>(0, 0), zeroColumn,
      JvLft.block<3, 7>(0, 3), zero7Columns;
  Eigen::Matrix<double, 3, 18> FullJacobianDerLft;
  FullJacobianDerLft << dJvLft.block<3, 3>(0, 0), zeroColumn,
      dJvLft.block<3, 7>(0, 3), zero7Columns;
  Eigen::MatrixXd PLeft = weightLeft * FullJacobianLeft;
  Eigen::VectorXd bLeft = -weightLeft * (FullJacobianDerLft * dq - ddxrefLft);

  // Left Arm Orientation
  setLeftOrientationOptParams(_LeftTargetRPY);

  // Right Arm Position
  Eigen::Vector3d xRgt = mRightEndEffector->getTransform().translation();
  Eigen::Vector3d dxRgt = mRightEndEffector->getLinearVelocity();
  math::LinearJacobian JvRgt = mRightEndEffector->getLinearJacobian();  // 3 x n
  math::LinearJacobian dJvRgt =
      mRightEndEffector->getLinearJacobianDeriv();  // 3 x n
  Eigen::Vector3d ddxrefRgt =
      -mKp * (xRgt - _RightTargetPosition) - mKv * dxRgt;
  Eigen::Matrix<double, 3, 18> FullJacobianRight;
  FullJacobianRight << JvRgt.block<3, 3>(0, 0), zeroColumn, zero7Columns,
      JvRgt.block<3, 7>(0, 3);
  Eigen::Matrix<double, 3, 18> FullJacobianDerRgt;
  FullJacobianDerRgt << dJvRgt.block<3, 3>(0, 0), zeroColumn, zero7Columns,
      dJvRgt.block<3, 7>(0, 3);
  Eigen::MatrixXd PRight = weightRight * FullJacobianRight;
  Eigen::VectorXd bRight = -weightRight * (FullJacobianDerRgt * dq - ddxrefRgt);

  // Right Arm Orientation
  setRightOrientationOptParams(_RightTargetRPY);

  // CoM
  setBalanceOptParams(thref, dthref, ddthref);
  /*double zCOM = mRobot->getCOM()(2);
  double dzCOM = mRobot->getCOMLinearVelocity()(2);
  Eigen::MatrixXd JzCOM = Eigen::MatrixXd::Zero(1, 18);
  JzCOM = mRobot->getCOMLinearJacobian().row(2);
  Eigen::MatrixXd dJzCOM = Eigen::MatrixXd::Zero(1, 18);
  dJzCOM = mRobot->getCOMLinearJacobianDeriv().row(2);
  Eigen::VectorXd ddzCOMref = Eigen::VectorXd::Zero(1);
  ddzCOMref(0) = -KpzCOM * zCOM - KvzCOM * dzCOM;
  Eigen::MatrixXd PBalance = weightBalance * JzCOM;
  Eigen::VectorXd bBalance = -weightBalance * (dJzCOM * dq - ddzCOMref);*/

  // Regulator
  Eigen::MatrixXd PRegulator =
      weightRegulator * Eigen::MatrixXd::Identity(dof, dof);
  Eigen::VectorXd bRegulator = -weightRegulator * 10 * dq;

  // Optimizer stuff
  OptParams optParams;
  Eigen::MatrixXd NewP(PRight.rows() + PLeft.rows() + PRegulator.rows() +
                           mPBal.rows() + mPOrL.rows() + mPOrR.rows(),
                       mOptDim);
  // clang-format off
  NewP << PRight.col(0), PRight.rightCols(mOptDim - 1),
      PLeft.col(0), PLeft.rightCols(mOptDim - 1),
      PRegulator.col(0), PRegulator.rightCols(mOptDim - 1),
      mPBal.col(0), mPBal.rightCols(mOptDim - 1),
      mPOrL.col(0), mPOrL.rightCols(mOptDim - 1),
      mPOrR.col(0), mPOrR.rightCols(mOptDim - 1);
  // clang-format on
  Eigen::VectorXd NewB(bRight.rows() + bLeft.rows() + bRegulator.rows() +
                           mbBal.rows() + mbOrL.rows() + mbOrR.rows(),
                       bRight.cols());
  NewB << bRight, bLeft, bRegulator, mbBal, mbOrL, mbOrR;
  optParams.P = NewP;
  optParams.b = NewB;
  nlopt::opt opt(nlopt::LD_SLSQP, mOptDim);
  std::vector<double> ddq_vec(mOptDim);
  Eigen::VectorXd::Map(&ddq_vec[0], mddqBodyRef.size()) = mddqBodyRef;
  double minf;
  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(ddq_vec, minf);
  for (int i = 0; i < mOptDim; i++) mddqBodyRef(i) = ddq_vec[i];

  // Torques
  Eigen::MatrixXd M = mRobot->getMassMatrix();                 // n x n
  Eigen::VectorXd Cg = mRobot->getCoriolisAndGravityForces();  // n x 1
  mMM << M(0, 0), M.topRightCorner(1, mOptDim - 1),
      M.bottomLeftCorner(mOptDim - 1, 1),
      M.bottomRightCorner(mOptDim - 1, mOptDim - 1);
  mhh << Cg(0), Cg.tail(mOptDim - 1);
  mForces = mMM * mddqBodyRef + mhh;
  Eigen::VectorXd forces = Eigen::VectorXd::Zero(dof);
  forces(0) = mForces(0);
  forces.tail(mOptDim -1) = mForces.tail(mOptDim - 1);
  mRobot->setForces(forces);
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const { return mRobot; }

//=========================================================================
dart::dynamics::BodyNode* Controller::getEndEffector(
    const std::string& s) const {
  if (!s.compare("left")) {
    return mLeftEndEffector;
  } else if (!s.compare("right")) {
    return mRightEndEffector;
  }
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {}
