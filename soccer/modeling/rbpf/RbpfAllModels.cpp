#if 0
/*
 * RbpfAllModels.cpp
 *
 *  See: RbpfAllModels.hpp for additional information.
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include <RbpfAllModels.hpp>

using namespace LinAlg;
// *****************************************************************************
// Constructor: RbpfModelRolling()
//   model for free rolling ball, ignores control input
//   state: X (6 x 1) = {x, y, vx, vy, ax, ay}
//   requires state size (n) = 6, control size (m) = 6, measurement size (s) = 2
//   initializes: F, H, Q, R
RbpfModelRolling::RbpfModelRolling(){
	// compute state transition Jacobian (df/dx) (n x n)
	computeTransitionJacobian(0);
	// compute observation Jacobian (dh/dx) (s x n)
	_H.setIdentity();
	// initialize process noise (n x n)
	double sP = 0.2, sV = 1.0, sA = 1000.0;
	_Q.setZero();
	_Q(0,0)=sP; _Q(1,1)=sP;
	_Q(2,2)=sV; _Q(3,3)=sV;
	_Q(4,4)=sA; _Q(5,5)=sA;
	// initialize measurement noise (s x s)
	_R.setIdentity();
	_R *= 0.1;
}
RbpfModelRolling::~RbpfModelRolling(){}
// Function: transitionModel(X, U, dt)
//   computes the effect of U and dt on the state, and stores the result in F
void RbpfModelRolling::transitionModel(VectorNf &X, VectorMf &U, double dt){
	X(0) = X(0) + X(2)*dt + 0.5*X(4)*dt*dt ; // f(x) = x + vx*dt + 1/2*ax*dt^2
	X(1) = X(1) + X(3)*dt + 0.5*X(5)*dt*dt ; // f(y) = y + vy*dt + 1/2*ay*dt^2
	X(2) = X(2) + X(4)*dt;                   // f(vx) = vx + ax*dt
	X(3) = X(3) + X(5)*dt;                   // f(vy) = vy + ay*dt
	X(4) = X(4);                             // f(ax) = ax
	X(5) = X(5);                             // f(ay) = ay
}
// Function: computeTransitionJacobian(dt)
//   computes the Jacobian of the transitionModel function, wrt the state and
//   control input. Requires that F has size (n x n)
//   Call before using the state transition Jacobian, F.
void RbpfModelRolling::computeTransitionJacobian(double dt){
	_F(0,0)=01; _F(0,1)=00; _F(0,2)=dt; _F(0,3)=00; _F(0,4)=0.5*dt*dt; _F(0,5)=00; // df/dx
	_F(1,0)=00; _F(1,1)=01; _F(1,2)=00; _F(1,3)=dt; _F(1,4)=00; _F(1,5)=0.5*dt*dt; // df/dy
	_F(2,0)=00; _F(2,1)=00; _F(2,2)=01; _F(2,3)=00; _F(2,4)=dt; _F(2,5)=00;        // df/dvx
	_F(3,0)=00; _F(3,1)=00; _F(3,2)=00; _F(3,3)=01; _F(3,4)=00; _F(3,5)=dt;        // df/dvy
	_F(4,0)=00; _F(4,1)=00; _F(4,2)=00; _F(4,3)=00; _F(4,4)=01; _F(4,5)=00;        // df/dax
	_F(5,0)=00; _F(5,1)=00; _F(5,2)=00; _F(5,3)=00; _F(5,4)=00; _F(5,5)=01;        // df/day
}
// Function: observationModel(&X, &out)
//   calculates naive observation of the first s components of X, storing the
//   result in out. For RoboCup, this will correspond to the x and y of the ball
void RbpfModelRolling::observationModel(VectorNf &X, VectorSf &out){
	out = X.block(0, 2);
//	for(int i=0; i<s; i++)
//		out(i) = X(i);
}
// Function: computeObservationJacobian(dt)
//   computes the Jacobian of the observationModel function, wrt the state.
//   Requires that H has size (s x n)
//   Because the observation model is static for this model, H is computed
//   at initialization and does not need to be re-computed here.
//   Call before using the observation Jacobian, H.
void RbpfModelRolling::computeObservationJacobian(double dt){
//	_H.setIdentity(); // TODO: find out whether this needed to be disabled
}




// *****************************************************************************
// Constructor: RbpfModelKicked()
//   kicked ball with high noise, ignores control input
//   Note: could be written to accept control input from our robots
//   state: X (6 x 1) = {x, y, vx, vy, ax, ay}
//   requires state size (n) = 6, control size (m) = 6, measurement size (s) = 2
//   initializes: F, H, Q, R
RbpfModelKicked::RbpfModelKicked(){
	// compute state transition Jacobian (df/dx) (n x n)
	computeTransitionJacobian(0);
	// compute observation Jacobian (dh/dx) (s x n)
	_H.setIdentity();
	// initialize process noise (n x n)
	double sP = 1.0, sV = 1.0, sA = 1000.0;
	_Q.setZero();
	_Q(0,0)=sP; _Q(1,1)=sP;
	_Q(2,2)=sV; _Q(3,3)=sV;
	_Q(4,4)=sA; _Q(5,5)=sA;
	// initialize measurement noise (s x s)
	_R.setIdentity();
	_R *= 0.1;
}
RbpfModelKicked::~RbpfModelKicked(){}
// Function: transitionModel(X, U, dt)
//   computes the effect of U and dt on the state, and stores the result in F
void RbpfModelKicked::transitionModel(VectorNf &X, VectorMf &U, double dt){
	X(0) = X(0) + X(2)*dt + 0.5*X(4)*dt*dt ; // f(x) = x + vx*dt + 1/2*ax*dt^2
	X(1) = X(1) + X(3)*dt + 0.5*X(5)*dt*dt ; // f(y) = y + vy*dt + 1/2*ay*dt^2
	X(2) = X(2) + X(4)*dt;                   // f(vx) = vx + ax*dt
	X(3) = X(3) + X(5)*dt;                   // f(vy) = vy + ay*dt
	X(4) = X(4);                             // f(ax) = ax
	X(5) = X(5);                             // f(ay) = ay
}
// Function: computeTransitionJacobian(dt)
//   computes the Jacobian of the transitionModel function, wrt the state and
//   control input. Requires that F has size (n x n)
//   Call before using the state transition Jacobian, F.
void RbpfModelKicked::computeTransitionJacobian(double dt){
	_F(0,0)=01; _F(0,1)=00; _F(0,2)=dt; _F(0,3)=00; _F(0,4)=0.5*dt*dt; _F(0,5)=00; // df/dx
	_F(1,0)=00; _F(1,1)=01; _F(1,2)=00; _F(1,3)=dt; _F(1,4)=00; _F(1,5)=0.5*dt*dt; // df/dy
	_F(2,0)=00; _F(2,1)=00; _F(2,2)=01; _F(2,3)=00; _F(2,4)=dt; _F(2,5)=00;        // df/dvx
	_F(3,0)=00; _F(3,1)=00; _F(3,2)=00; _F(3,3)=01; _F(3,4)=00; _F(3,5)=dt;        // df/dvy
	_F(4,0)=00; _F(4,1)=00; _F(4,2)=00; _F(4,3)=00; _F(4,4)=01; _F(4,5)=00;        // df/dax
	_F(5,0)=00; _F(5,1)=00; _F(5,2)=00; _F(5,3)=00; _F(5,4)=00; _F(5,5)=01;        // df/day
}
// Function: observationModel(&X, &out)
//   calculates naive observation of the first s components of X, storing the
//   result in out. For RoboCup, this will correspond to the x and y of the ball
void RbpfModelKicked::observationModel(Vector &X, Vector &out){
	out = X.block(0, 2);
//	for(int i=0; i<s; i++)
//		out(i) = X(i);
}
// Function: computeObservationJacobian(dt)
//   computes the Jacobian of the observationModel function, wrt the state.
//   Requires that H has size (s x n)
//   Because the observation model is static for this model, H is computed
//   at initialization and does not need to be re-computed here.
//   Call before using the observation Jacobian, H.
void RbpfModelKicked::computeObservationJacobian(double dt){
//	_H.setIdentity(); // TODO: find out whether this needed to be disabled
}




// *****************************************************************************
// Constructor: RbpfModelRollingFriction()
//   model for rolling ball with friction, ignores control input
//   state: X (6 x 1) = {x, y, vx, vy, ax, ay}
//   requires state size (n) = 6, control size (m) = 6, measurement size (s) = 2
//   initializes: F, H, Q, R
RbpfModelRollingFriction::RbpfModelRollingFriction(){
	// compute state transition Jacobian (df/dx) (n x n)
	computeTransitionJacobian(0);
	// compute observation Jacobian (dh/dx) (s x n)
	_H.setIdentity();
	// initialize process noise (n x n)
	double sP = 0.01, sV = 10.0, sA = 30.0;
	_Q.setZero();
	_Q(0,0)=sP; _Q(1,1)=sP;
	_Q(2,2)=sV; _Q(3,3)=sV;
	_Q(4,4)=sA; _Q(5,5)=sA;
	// initialize measurement noise (s x s)
	_R.setIdentity();
	_R *= 0.01;
}
RbpfModelRollingFriction::~RbpfModelRollingFriction(){}
// Function: transitionModel(X, U, dt)
//   computes the effect of U and dt on the state, and stores the result in F
void RbpfModelRollingFriction::transitionModel(VectorNf &X, VectorMf &U, double dt){
	double rollingFriction = -0.001; // F/m
	X(0) = X(0) + X(2)*dt + 0.5*X(4)*dt*dt ; // f(x) = x + vx*dt + 1/2*ax*dt^2
	X(1) = X(1) + X(3)*dt + 0.5*X(5)*dt*dt ; // f(y) = y + vy*dt + 1/2*ay*dt^2
	X(2) = X(2) + X(4)*dt;                   // f(vx) = vx + ax*dt
	X(3) = X(3) + X(5)*dt;                   // f(vy) = vy + ay*dt
	X(4) = abs(X(4) + rollingFriction);      // f(ax) = ax
	X(5) = abs(X(5) + rollingFriction);      // f(ay) = ay
}
// Function: computeTransitionJacobian(dt)
//   computes the Jacobian of the transitionModel function, wrt the state and
//   control input. Requires that F has size (n x n)
//   Call before using the state transition Jacobian, F.
void RbpfModelRollingFriction::computeTransitionJacobian(double dt){
	_F(0,0)=01; _F(0,1)=00; _F(0,2)=dt; _F(0,3)=00; _F(0,4)=0.5*dt*dt; _F(0,5)=00; // df/dx
	_F(1,0)=00; _F(1,1)=01; _F(1,2)=00; _F(1,3)=dt; _F(1,4)=00; _F(1,5)=0.5*dt*dt; // df/dy
	_F(2,0)=00; _F(2,1)=00; _F(2,2)=01; _F(2,3)=00; _F(2,4)=dt; _F(2,5)=00;        // df/dvx
	_F(3,0)=00; _F(3,1)=00; _F(3,2)=00; _F(3,3)=01; _F(3,4)=00; _F(3,5)=dt;        // df/dvy
	_F(4,0)=00; _F(4,1)=00; _F(4,2)=00; _F(4,3)=00; _F(4,4)=01; _F(4,5)=00;        // df/dax
	_F(5,0)=00; _F(5,1)=00; _F(5,2)=00; _F(5,3)=00; _F(5,4)=00; _F(5,5)=01;        // df/day
}
// Function: observationModel(&X, &out)
//   calculates naive observation of the first s components of X, storing the
//   result in out. For RoboCup, this will correspond to the x and y of the ball
void RbpfModelRollingFriction::observationModel(VectorNf &X, VectorSf &out){
	out = X.head<s>();
//	for(int i=0; i<s; i++)
//		out(i) = X(i);
}
// Function: computeObservationJacobian(dt)
//   computes the Jacobian of the observationModel function, wrt the state.
//   Requires that H has size (s x n)
//   Because the observation model is static for this model, H is computed
//   at initialization and does not need to be re-computed here.
//   Call before using the observation Jacobian, H.
void RbpfModelRollingFriction::computeObservationJacobian(double dt){
//	_H.setIdentity(); // TODO: find out if this needed to be disabled
	//H(0,0)=1; H(0,1)=0; H(0,2)=0; H(0,3)=0; H(0,4)=0; H(0,5)=0; // dh(X)/dx
	//H(1,0)=0; H(1,1)=1; H(1,2)=0; H(1,3)=0; H(1,4)=0; H(1,5)=0; // dh(X)/dy
}
#endif
