/*
 * RbpfModel.cpp
 *
 *  See: RbpfModel.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include "RbpfModel.hpp"

using namespace LinAlg;
using namespace rbpf;

// initializes n, m, s, F, H, Q, R, and space used for intermediates
// calculations: Inn (n x n Identity matrix), h, Yhat, S
RbpfModel::RbpfModel(Modeling::RobotModel::RobotMap *robotMap)
: _robotMap(robotMap), _Inn(MatrixNNf::Identity()),
  _h(VectorSf::Zero()), _Yhat(VectorSf::Zero()), _S(MatrixSSf::Zero())
{
}

// performs EKF predict, storing the result in X and P
// X: state vector that will be updated (n x 1)
// P: state covariance that will be updated (n x n)
// U: control input (m x 1)
// dt: change in time
// TODO: for speed, computeTransitionJacobian can be performed once per model.
// TODO: for speed, the transpose of F can be computed once per model.
// TODO: for speed, temp matrices should be removed.
void RbpfModel::predict(VectorNf &X, MatrixNNf &P, VectorMf &U, double dt){
	// Xhat = f(X,U,dt), f() = state transition model
	// P = F*P*F' + Q
	computeTransitionJacobian(dt); // recompute F
	transitionModel(X, U, dt); // X = f(X,U,dt)
	P = _F * P * _F.transpose() + _Q;
}

// performs EKF update, storing the result in X and P
// X: state vector that will be updated (n x 1)
// P: state covariance that will be updated (n x n)
// Z: observation (s x 1)
// dt: change in time
// TODO: for speed, the transpose of H can be computed once per model.
// TODO: for speed, temp matrices should be removed.
void RbpfModel::update(VectorNf &X, MatrixNNf &P, VectorSf &Z, double dt){
	// Yhat = Z - h(Xhat), h() = observation model
	// S = H*P*H' + R
	// K = P*H'*S^{-1}
	// Xhat = Xhat + K*Yhat
	// P = (I - K*H)*P
	computeObservationJacobian(dt); // recompute H
	observationModel(X,_h); // h = h(Xhat)
	_Yhat = Z-_h;
	_S = _H*P*_H.transpose() + _R;

	Matrix K = P * _H.transpose() * _S.inverse();
	X += K * _Yhat;
	P = (_Inn - K * _H) * P;

	// TODO: do we really ever have a case where this is not invertible?
//	if(InvertMatrix(S,Sinv)){
//		K = prod(K,Sinv);
//		X = X + prod(K,Yhat);
//		Matrix temp = Inn - prod(K,H);
//		P = prod(temp,P);
//	}else{
//		// S^{-1} could not be determined, throw exception
//		printf("S^-1 could not be calculated during EKF update in RbpfModel.update()");
//		bool inverseFound = false;
//		assert(inverseFound);
//	}
}

// overloaded operator for printing a model (debugging)
// prints: F, H, Q, and R
std::ostream& operator<<(std::ostream& out, const RbpfModel &model){
	out << "F:" << std::endl;
	for(size_t x=0; x<model.n; x++){
		out << "\t";
		for(size_t y=0; y<model.n; y++){out << model._F(x,y) << ", ";}
		out << std::endl;
	}
	out << std::endl << "H:" << std::endl;
	for(size_t x=0; x<model.s; x++){
		out << "\t";
		for(size_t y=0; y<model.n; y++){out << model._H(x,y) << ", ";}
		out << std::endl;
	}
	out << std::endl << "Q:" << std::endl;
	for(size_t x=0; x<model.n; x++){
		out << "\t";
		for(size_t y=0; y<model.n; y++){out << model._Q(x,y) << ", ";}
		out << std::endl;
	}
	out << std::endl << "R:" << std::endl;
	for(size_t x=0; x<model.s; x++){
		out << "\t";
		for(size_t y=0; y<model.s; y++){out << model._R(x,y) << ", ";}
		out << std::endl;
	}
	return out;
}
