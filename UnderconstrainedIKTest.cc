#include "UCInvKinematics.hh"

#include <Eigen/Dense>
#include <string>
#include <vector>

struct MyKinematicModel {
  
};

struct MyKinematicChainPt {
  std::string linkName;
  Eigen::Vector3d linkRelPos;
};

int main() {

  int nJoints = 10;
  double gradStep = 1e-1;

  // initialize the joint IK states
  std::vector<UCJointIKState> jointIKStates(nJoints);
  for (int ii = 0; ii < jointIKStates.size(); ii++) {
    jointIKStates[ii].index = ii;
    jointIKStates[ii].isKnown = false;
    jointIKStates[ii].angle = 0.0;
  }

  // the point on the kinematic chain for which we are solving IK
  MyKinematicChainPt kinChainPt;
  kinChainPt.linkName = "forearm";
  kinChainPt.linkRelPos(0) = 0;
  kinChainPt.linkRelPos(1) = 0;
  kinChainPt.linkRelPos(2) = 0;

  // the body-frame coordinates of the point for which we are solving IK
  Eigen::Vector3d kinChainPtBodyPos;
  kinChainPtBodyPos(0) = 10;
  kinChainPtBodyPos(1) = 0;
  kinChainPtBodyPos(2) = 0;

  // initialize the kinematic model
  MyKinematicModel kinModel;

  // call the IK solver
  UCIKSolver<MyKinematicModel, MyKinematicChainPt>::
    SolveIKPrior(kinModel, jointIKStates, 
                 kinChainPt, kinChainPtBodyPos,
                 gradStep);

};
