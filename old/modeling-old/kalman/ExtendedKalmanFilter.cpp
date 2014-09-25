#include "ExtendedKalmanFilter.hpp"

using namespace LinAlg;

Modeling::ExtendedKalmanFilter::ExtendedKalmanFilter(const Matrix &Q, const Matrix &R)
: _X(4,1), _P(4,4), _F(4,4), _H(2,4), _Q(Q),
  _R(R), _Inn(4,4), _h(2), _Yhat(2,1), _S(2,2)
{
	_X.clear(); _P.clear(); _F.clear(); _H.clear(); // zero out matrices
	_Inn.clear(); _h.clear(); _Yhat.clear(); _S.clear();
	for(int i=0;i<4;i++){_Inn(i,i)=1.0;} // initialize identity matrix
	computeObservationJacobian();
}

Modeling::ExtendedKalmanFilter::~ExtendedKalmanFilter() {

}

// performs EKF predict, storing the result in X and P
void Modeling::ExtendedKalmanFilter::predict(double dt){
	// Xhat = f(X,dt), f() = state transition model
	applyTransitionModel(dt); // compute updated state
	// P = F*P*F' + Q
	computeTransitionJacobian(dt); // compute F = dF/dX
	Matrix FPFt = ublas::prod(_F,_P);
	FPFt = ublas::prod(FPFt,ublas::trans(_F));
	_P = FPFt + _Q;
}

// performs EKF update, comparing the predicted state to the actual state
// and correcting the internal state accordingly. Call predict before using.
void Modeling::ExtendedKalmanFilter::update(Vector &Z){
	// Yhat = Z - h(Xhat), h() = observation model
	applyObservationModel(); // h = h(Xhat)
	_Yhat = (Z - _h);
	// S = H*P*H' + R
	// K = P*H'*S^{-1}
	// Xhat = Xhat + K*Yhat
	// P = (I - K*H)*P
	_S = ublas::prod(_H,_P);
	_S = ublas::prod(_S,ublas::trans(_H)) + _R;
	Matrix K = ublas::prod(_P,ublas::trans(_H));
	Matrix Sinv(2,2);
	Sinv.clear();
	if(InvertMatrix(_S,Sinv)){
		K = ublas::prod(K,Sinv);
		_X = _X + ublas::prod(K,_Yhat);
		_P = ublas::prod(_Inn - ublas::prod(K,_H),_P);
	}else{
		// S^{-1} could not be determined, throw exception
		throw std::runtime_error("S^{-1} could not be determined in Extended Kalman Filter");
	}
}

// predicts the state up to the time the observation was taken, then updates
// the filter based on the new observation.
void Modeling::ExtendedKalmanFilter::update(Vector &Z, double dt){
	predict(dt);
	update(Z);
}

// computes the Jacobian of the transition function, wrt the state
void Modeling::ExtendedKalmanFilter::computeTransitionJacobian(double dt){
	_F(0,0)=01; _F(0,1)=00; _F(0,2)=dt; _F(0,3)=00; // dF/dx
	_F(1,0)=00; _F(1,1)=01; _F(1,2)=00; _F(1,3)=dt; // dF/dy
	_F(2,0)=00; _F(2,1)=00; _F(2,2)=01; _F(2,3)=00; // dF/dvx
	_F(3,0)=00; _F(3,1)=00; _F(3,2)=00; _F(3,3)=01; // dF/dvy
}

// computes the effect of dt on the state
void Modeling::ExtendedKalmanFilter::applyTransitionModel(double dt){
	_X(0) = _X(0) + _X(2)*dt; // f(x) = x + vx*dt
	_X(1) = _X(1) + _X(3)*dt; // f(y) = y + vy*dt
}

// computes the Jacobian of the observation function, wrt the state
void Modeling::ExtendedKalmanFilter::computeObservationJacobian(){
	_H(0,0)=1; _H(0,1)=0; _H(0,2)=0; _H(0,3)=0; // dh(X)/dx
	_H(1,0)=0; _H(1,1)=1; _H(1,2)=0; _H(1,3)=0; // dh(X)/dy
}

// computes the observation (h) of the state (X)
void Modeling::ExtendedKalmanFilter::applyObservationModel(){
	_h(0) = _X(0);
	_h(1) = _X(1);
}
