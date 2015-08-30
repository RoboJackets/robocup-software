/*
 * RbpfModelKicked.cpp
 *
 *  See: RbpfModelKicked.hpp for additional information.
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include "RbpfModelKicked.hpp"
#include <Constants.hpp>

using namespace LinAlg;
using namespace rbpf;
using namespace Modeling;

// kicked ball with high noise, ignores control input
// Note: could be written to accept control input from our robots
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
// requires state size (n) = 6, control size (m) = 6, measurement size (s) = 2
// initializes: F, H, Q, R
RbpfModelKicked::RbpfModelKicked(RobotModel::RobotMap* _robotMap,
                                 Configuration* config)
    : RbpfModel(_robotMap),
      _processNoiseSqrdPos(config->createDouble(
          "rbpfModelBallKicked/Process Noise Position", 1.0)),
      _processNoiseSqrdVel(config->createDouble(
          "rbpfModelBallKicked/Process Noise Velocity", 1.0)),
      _processNoiseSqrdAcc(config->createDouble(
          "rbpfModelBallKicked/Process Noise Acceleration", 1000.0)),
      _measurementNoiseSqrd(config->createDouble(
          "rbpfModelBallKicked/Measurement Noise Position", 0.01)) {
    // compute state transition Jacobian (df/dx) (n x n)
    computeTransitionJacobian(0);
    // compute observation Jacobian (dh/dx) (s x n)
    _H.setIdentity();
    //	H(0,0)=1; H(0,1)=0; H(0,2)=0; H(0,3)=0; H(0,4)=0; H(0,5)=0; // dh(X)/dx
    //	H(1,0)=0; H(1,1)=1; H(1,2)=0; H(1,3)=0; H(1,4)=0; H(1,5)=0; // dh(X)/dy

    // initialize parameters
    initParams();
}

void RbpfModelKicked::initializeQ() {
    double sP = *_processNoiseSqrdPos;
    double sV = *_processNoiseSqrdVel;
    double sA = *_processNoiseSqrdAcc;
    _Q(0, 0) = sP;
    _Q(1, 1) = sP;
    _Q(2, 2) = sV;
    _Q(3, 3) = sV;
    _Q(4, 4) = sA;
    _Q(5, 5) = sA;
    //	Q(0,0)=sP; Q(0,1)=00; Q(0,2)=00; Q(0,3)=00; Q(0,4)=00; Q(0l,5)=00;
    //	Q(1,0)=00; Q(1,1)=sP; Q(1,2)=00; Q(1,3)=00; Q(1,4)=00; Q(1,5)=00;
    //	Q(2,0)=00; Q(2,1)=00; Q(2,2)=sV; Q(2,3)=00; Q(2,4)=00; Q(2,5)=00;
    //	Q(3,0)=00; Q(3,1)=00; Q(3,2)=00; Q(3,3)=sV; Q(3,4)=00; Q(3,5)=00;
    //	Q(4,0)=00; Q(4,1)=00; Q(4,2)=00; Q(4,3)=00; Q(4,4)=sA; Q(4,5)=00;
    //	Q(5,0)=00; Q(5,1)=00; Q(5,2)=00; Q(5,3)=00; Q(5,4)=00; Q(5,5)=sA;
}

void RbpfModelKicked::initializeR() {
    double sM = *_measurementNoiseSqrd;
    _R.setIdentity();
    _R *= sM;
}

void RbpfModelKicked::initParams() {
    initializeQ();
    initializeR();
}

RbpfModelKicked::~RbpfModelKicked() {}

// computes the effect of U and dt on the state, and stores the result in F
void RbpfModelKicked::transitionModel(VectorNd& X, const VectorMd& U,
                                      double dt) const {
    X(0) = X(0) + X(2) * dt +
           0.5 * X(4) * dt * dt;  // f(x) = x + vx*dt + 1/2*ax*dt^2
    X(1) = X(1) + X(3) * dt +
           0.5 * X(5) * dt * dt;  // f(y) = y + vy*dt + 1/2*ay*dt^2
    X(2) = X(2) + X(4) * dt;      // f(vx) = vx + ax*dt
    X(3) = X(3) + X(5) * dt;      // f(vy) = vy + ay*dt
    //	X(4) = X(4);                             // f(ax) = ax
    //	X(5) = X(5);                             // f(ay) = ay
}

// computes the Jacobian of the transitionModel function, wrt the state and
// control input. Requires that F has size (n x n)
// Call before using the state transition Jacobian, F.
void RbpfModelKicked::computeTransitionJacobian(double dt) {
    _F(0, 0) = 01;
    _F(0, 1) = 00;
    _F(0, 2) = dt;
    _F(0, 3) = 00;
    _F(0, 4) = 0.5 * dt * dt;
    _F(0, 5) = 00;  // df/dx
    _F(1, 0) = 00;
    _F(1, 1) = 01;
    _F(1, 2) = 00;
    _F(1, 3) = dt;
    _F(1, 4) = 00;
    _F(1, 5) = 0.5 * dt * dt;  // df/dy
    _F(2, 0) = 00;
    _F(2, 1) = 00;
    _F(2, 2) = 01;
    _F(2, 3) = 00;
    _F(2, 4) = dt;
    _F(2, 5) = 00;  // df/dvx
    _F(3, 0) = 00;
    _F(3, 1) = 00;
    _F(3, 2) = 00;
    _F(3, 3) = 01;
    _F(3, 4) = 00;
    _F(3, 5) = dt;  // df/dvy
    _F(4, 0) = 00;
    _F(4, 1) = 00;
    _F(4, 2) = 00;
    _F(4, 3) = 00;
    _F(4, 4) = 01;
    _F(4, 5) = 00;  // df/dax
    _F(5, 0) = 00;
    _F(5, 1) = 00;
    _F(5, 2) = 00;
    _F(5, 3) = 00;
    _F(5, 4) = 00;
    _F(5, 5) = 01;  // df/day
}

// calculates naive observation of the first s components of X, storing the
// result in out. For RoboCup, this will correspond to the x and y of the ball
void RbpfModelKicked::observationModel(const VectorNd& X, VectorSd& out) const {
    //	out = X.head<2>();
    out(0) = X(0);
    out(1) = X(1);
}

// computes the Jacobian of the observationModel function, wrt the state.
// equires that H has size (s x n)
// Because the observation model is static for this model, H is computed
// at initialization and does not need to be re-computed here.
// Call before using the observation Jacobian, H.
void RbpfModelKicked::computeObservationJacobian(double dt) {
    // _H.setIdentity();
    // H(0,0)=1; H(0,1)=0; H(0,2)=0; H(0,3)=0; H(0,4)=0; H(0,5)=0; // dh(X)/dx
    // H(1,0)=0; H(1,1)=1; H(1,2)=0; H(1,3)=0; H(1,4)=0; H(1,5)=0; // dh(X)/dy
}

// checks whether close to other robots. If so, assumes a kick and
// updates the velocity of the ball. Otherwise, standard EKF update.
// Note: does not consider orientation of robots, so kicking
//       is possible from any direction.
// Need to consider control input here.
void RbpfModelKicked::update(VectorNd& X, MatrixNNd& P, const VectorSd& Z,
                             double dt) {
    Geometry2d::Point bPos(X(0), X(1));
    Geometry2d::Point rPos;
    bool robotKicked = false;
    for (RobotModel::RobotMap::const_iterator r = _robotMap->begin();
         r != _robotMap->end(); r++) {
        rPos = r->second->pos();
        if (bPos.distTo(rPos) < Robot_Radius) {
            robotKicked = true;
            break;
        }
    }
    if (robotKicked) {
        Geometry2d::Point kickVel(bPos - rPos);
        kickVel = kickVel.normalized();
        kickVel = kickVel * 5;
        X(0) = rPos.x + kickVel.x * dt;
        X(1) = rPos.y + kickVel.y * dt;
        X(2) = kickVel.x;
        X(3) = kickVel.y;
    } else {
        RbpfModel::update(X, P, Z, dt);
    }
}
