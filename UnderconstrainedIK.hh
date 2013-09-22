#ifndef __UCInvKinematics_hh__
#define __UCInvKinematics_hh__

#include <Eigen/Dense>
#include <vector>

struct UCJointIKState { 
  int index;                  // index of the joint
  bool isKnown;               // true iff. the joint angle is known
  double angle;               // the joint's known (or guessed) angle
};

typedef std::vector<UCJointIKState> UCJointIKStateVector;

template <typename KinModel, typename KinChainPt>
class KinematicModelOps {
  
public:

  static 
  Eigen::Vector3d 
  ComputeCartesianFK(KinModel const& kinModel, 
                     KinChainPt const& kinChainPt,
                     std::vector<UCJointIKState> const& jointIKStates);

  static 
  Eigen::MatrixXd 
  ComputeJacobian(KinModel const& kinModel, 
                  KinChainPt const& kinChainPt,
                  std::vector<UCJointIKState> const& jointIKStates);

};

/**
   \tparam KinModel The type of the robot's kinematic model
   \tparam KinChainPt The type of a point wrt. the robot's kinematic chain
 */
template <typename KinModel, typename KinChainPt>
class UCIKSolver {

public:

  /**
     Solves IK with an optional prior penalizing deviation from 
     a given set of joint angles.  

     \param kinModel The robot's kinematic model
     \param jointIKStates Vector of IK states for each joint in group
     \param kinChainPt A point on the robot's kinematic chain
     \param kinChainPtBodyPos Body coordinates of the point
     \param step Determines gradient descent step size
     \param tol The tolerated distance error 
     \param priorReg The deviation penalty parameter
     \param qPrior The joint angles wrt. which the penalty is computed
  */
  static 
  UCJointIKStateVector
  SolveIKPrior(KinModel const& kinModel,
               UCJointIKStateVector const& jointIKStates,
               KinChainPt const& kinChainPt,
               Eigen::Vector3d const& kinChainPtBodyPos, 
               double step,
               double tol = 1e-3,
               double priorReg = 0.0,
               std::vector<double> const& qPrior = std::vector<double>());

  /**
     Solves IK for an entire path. Optional smoothness prior penalizes
     deviation between successive IK solutions.

     \param kinModel The robot's kinematic model
     \param initialJointIKState Initial IK state for each joint
     \param kinChainPt A point on the robot's kinematic chain
     \param kinChainPtBodyPosPath Sequence of body coordinates of the point 
     \param step Determines gradient descent step size
     \param tol The tolerated distance error 
     \param priorReg The deviation penalty parameter
   */
  static
  std::vector<UCJointIKStateVector>
  SolveIKPath(KinModel const& kinModel,
              UCJointIKStateVector const& initialJointIKState,
              KinChainPt const& kinChainPt,
              std::vector<Eigen::Vector3d> const& kinChainPtBodyPosPath, 
              double step,
              double tol = 1e-3,
              double priorReg = 0.0);

};

template <typename KinModel, typename KinChainPt>
UCJointIKStateVector
UCIKSolver<KinModel,KinChainPt>::SolveIKPrior
(KinModel const& kinModel,
 UCJointIKStateVector const& jointIKStates,
 KinChainPt const& kinChainPt,
 Eigen::Vector3d const& kinChainPtBodyPos,
 double step,
 double tol,
 double priorReg,
 std::vector<double> const& qPrior) {

  // compute FK for known point on kinematic chain
  Eigen::Vector3d const curPos = 
    KinematicModelOps<KinModel, KinChainPt>::
    ComputeCartesianFK(kinModel, kinChainPt, jointIKStates);

  // error in FK position
  Eigen::MatrixXd const deltaPos = curPos - kinChainPtBodyPos;

  // compute objective, terminate if necessary
  if (deltaPos.norm() < tol) return jointIKStates;

  // compute Jacobian wrt. unknown joint
  Eigen::MatrixXd dwdq = 
    KinematicModelOps<KinModel, KinChainPt>::
    ComputeJacobian(kinModel, kinChainPt, jointIKStates);
  Eigen::MatrixXd const dxdq = dwdq.block(0, 0, 3, jointIKStates.size());

  // compute gradient step
  Eigen::MatrixXd grad = 2 * deltaPos * dxdq;

  // compute contribution to gradient due to prior deviation penalty
  if (priorReg > 0.) {
    assert(qPrior.size() == jointIKStates.size());
    Eigen::RowVectorXd priorDeviation(qPrior.size());
    for (int ii = 0; ii < qPrior.size(); ii++) 
      priorDeviation(ii) = jointIKStates[ii].angle - qPrior[ii];
    grad = grad - (2 * priorReg * priorDeviation);
  }
  
  // update guesses 
  std::vector<UCJointIKState> newJointIKStates(jointIKStates);
  for (int iGrad = 0; iGrad < grad.size(); iGrad++) {
    if (!jointIKStates[iGrad].isKnown) 
      newJointIKStates[iGrad].angle -= step * grad(iGrad);
  }

  // recurse
  return SolveIKPrior(kinModel,
                      newJointIKStates,
                      kinChainPt,
                      kinChainPtBodyPos,
                      step,
                      tol, 
                      priorReg,
                      qPrior);

}

template <typename KinModel, typename KinChainPt>
std::vector<UCJointIKStateVector>
SolveIKPath(KinModel const& kinModel,
            UCJointIKStateVector const& initialJointIKStates,
            KinChainPt const& kinChainPt,
            std::vector<Eigen::Vector3d> const& kinChainPtBodyPosPath, 
            double step,
            double tol,
            double priorReg) {

  // compute the initial IK solution
  UCJointIKStateVector const jointIKStates1 = 
    UCIKSolver<KinModel,KinChainPt>
    ::SolveIKPrior(kinModel, initialJointIKStates, kinChainPt,
                   kinChainPtBodyPosPath[0], step, tol, 0.0);

  // store the initial IK solution as a prior
  std::vector<double> qPrior(initialJointIKStates.size());
  for (int ii = 0; ii < qPrior.size(); ii++)
    qPrior[ii] = jointIKStates1[ii].angle;

  // loop over input path, computing IK
  UCJointIKStateVector curJointIKState(initialJointIKStates);
  std::vector<UCJointIKStateVector> ikStatePath;

  for (int iPath = 0; iPath < kinChainPtBodyPosPath.size(); iPath++) {
    UCJointIKStateVector newIK =
      UCIKSolver<KinModel,KinChainPt>
      ::SolveIKPrior(kinModel, curJointIKState, kinChainPt,
                     kinChainPtBodyPosPath[iPath], step, tol, 
                     priorReg, qPrior);
    for (int ii = 0; ii < qPrior.size(); ii++) 
      qPrior[ii] = newIK[ii].angle;
    ikStatePath.push_back(newIK);
  }

  return ikStatePath;
}

#endif
