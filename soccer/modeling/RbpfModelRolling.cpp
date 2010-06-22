/*
 * RbpfModelRolling.cpp
 *
 *  See: RbpfModelRolling.hpp for additional information.
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include <RbpfModelRolling.hpp>

// model for free rolling ball, ignores control input
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
// requires state size (n) = 6, control size (m) = 6, measurement size (s) = 2
// initializes: F, H, Q, R
RbpfModelRolling::RbpfModelRolling(Modeling::RobotModel::RobotMap *_robotMap, ConfigFile::shared_worldmodel& cfg)
: RbpfModel(_robotMap), _config(cfg){
	assert(n==6); // state size (n) must = 6. If n changed, re-write this!
	assert(m==6); // control size (m) must = 6. If m changed, re-write this!
	assert(s==2); // measurement size (s) must = 2. If s changed, re-write this!
	// compute state transition Jacobian (df/dx) (n x n)
	computeTransitionJacobian(0);
	// compute observation Jacobian (dh/dx) (s x n)
	H(0,0)=1; H(0,1)=0; H(0,2)=0; H(0,3)=0; H(0,4)=0; H(0,5)=0; // dh(X)/dx
	H(1,0)=0; H(1,1)=1; H(1,2)=0; H(1,3)=0; H(1,4)=0; H(1,5)=0; // dh(X)/dy

	// initialize parameters
	initParams();
}

void RbpfModelRolling::initializeQ() {
	double sP = _config->rbpfModelBallRolling.processNoiseSqrdPos;
	double sV = _config->rbpfModelBallRolling.processNoiseSqrdVel;
	double sA = _config->rbpfModelBallRolling.processNoiseSqrdAcc;
	Q(0,0)=sP; Q(0,1)=00; Q(0,2)=00; Q(0,3)=00; Q(0,4)=00; Q(0,5)=00;
	Q(1,0)=00; Q(1,1)=sP; Q(1,2)=00; Q(1,3)=00; Q(1,4)=00; Q(1,5)=00;
	Q(2,0)=00; Q(2,1)=00; Q(2,2)=sV; Q(2,3)=00; Q(2,4)=00; Q(2,5)=00;
	Q(3,0)=00; Q(3,1)=00; Q(3,2)=00; Q(3,3)=sV; Q(3,4)=00; Q(3,5)=00;
	Q(4,0)=00; Q(4,1)=00; Q(4,2)=00; Q(4,3)=00; Q(4,4)=sA; Q(4,5)=00;
	Q(5,0)=00; Q(5,1)=00; Q(5,2)=00; Q(5,3)=00; Q(5,4)=00; Q(5,5)=sA;
}

void RbpfModelRolling::initializeR() {
	double sM = _config->rbpfModelBallRolling.measurementNoiseSqrd;
	R(0,0)=sM; R(0,1)=00;
	R(1,0)=00; R(1,1)=sM;
}

void RbpfModelRolling::initParams() {
	initializeQ();
	initializeR();
}

RbpfModelRolling::~RbpfModelRolling(){}

// computes the effect of U and dt on the state, and stores the result in F
void RbpfModelRolling::transitionModel(Vector &X, Vector &U, double dt){
	assert((int)X.size() == n); // X size must = 6. If changed, re-write this!
	X(0) = X(0) + X(2)*dt + 0.5*X(4)*dt*dt ; // f(x) = x + vx*dt + 1/2*ax*dt^2
	X(1) = X(1) + X(3)*dt + 0.5*X(5)*dt*dt ; // f(y) = y + vy*dt + 1/2*ay*dt^2
	X(2) = X(2) + X(4)*dt;                   // f(vx) = vx + ax*dt
	X(3) = X(3) + X(5)*dt;                   // f(vy) = vy + ay*dt
	X(4) = X(4);                             // f(ax) = ax
	X(5) = X(5);                             // f(ay) = ay
}

// computes the Jacobian of the transitionModel function, wrt the state and
// control input. Requires that F has size (n x n)
// Call before using the state transition Jacobian, F.
void RbpfModelRolling::computeTransitionJacobian(double dt){
	assert(n == 6); // n must be of size 6. If n changed, re-write this!
	F(0,0)=01; F(0,1)=00; F(0,2)=dt; F(0,3)=00; F(0,4)=0.5*dt*dt; F(0,5)=00; // df/dx
	F(1,0)=00; F(1,1)=01; F(1,2)=00; F(1,3)=dt; F(1,4)=00; F(1,5)=0.5*dt*dt; // df/dy
	F(2,0)=00; F(2,1)=00; F(2,2)=01; F(2,3)=00; F(2,4)=dt; F(2,5)=00;        // df/dvx
	F(3,0)=00; F(3,1)=00; F(3,2)=00; F(3,3)=01; F(3,4)=00; F(3,5)=dt;        // df/dvy
	F(4,0)=00; F(4,1)=00; F(4,2)=00; F(4,3)=00; F(4,4)=01; F(4,5)=00;        // df/dax
	F(5,0)=00; F(5,1)=00; F(5,2)=00; F(5,3)=00; F(5,4)=00; F(5,5)=01;        // df/day
}

// calculates naive observation of the first s components of X, storing the
// result in out. For RoboCup, this will correspond to the x and y of the ball
void RbpfModelRolling::observationModel(Vector &X, Vector &out){
	for(int i=0; i<s; i++)
		out(i) = X(i);
}

// computes the Jacobian of the observationModel function, wrt the state.
// Requires that H has size (s x n)
// Because the observation model is static for this model, H is computed
// at initialization and does not need to be re-computed here.
// Call before using the observation Jacobian, H.
void RbpfModelRolling::computeObservationJacobian(double dt){
	assert(n==6); // n must be of size 4. If n changed, re-write this!
	assert(s==2); // s must be of size 2. If s changed, re-write this!
	//H(0,0)=1; H(0,1)=0; H(0,2)=0; H(0,3)=0; H(0,4)=0; H(0,5)=0; // dh(X)/dx
	//H(1,0)=0; H(1,1)=1; H(1,2)=0; H(1,3)=0; H(1,4)=0; H(1,5)=0; // dh(X)/dy
}
