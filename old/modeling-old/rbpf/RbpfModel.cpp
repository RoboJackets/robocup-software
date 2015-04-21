/*
 * RbpfModel.cpp
 *
 *  See: RbpfModel.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include <iostream>
#include "RbpfModel.hpp"

using namespace std;
using namespace LinAlg;
using namespace rbpf;

// initializes n, m, s, F, H, Q, R, and space used for intermediates
// calculations: Inn (n x n Identity matrix), h, Yhat, S
RbpfModel::RbpfModel(Modeling::RobotModel::RobotMap *robotMap)
: _robotMap(robotMap), _Inn(MatrixNNd::Identity()),
  _h(VectorSd::Zero()), _Yhat(VectorSd::Zero()), _S(MatrixSSd::Zero())
{
}

// performs EKF predict, storing the result in X and P
// X: state vector that will be updated (n x 1)
// P: state covariance that will be updated (n x n)
// U: control input (m x 1)
// dt: change in time
// TODO: for speed, computeTransitionJacobian can be performed once per model.
// TODO: for speed, the transpose of F can be computed once per model.
void RbpfModel::predict(VectorNd &X, MatrixNNd &P, const VectorMd &U, double dt) const {
	// Xhat = f(X,U,dt), f() = state transition model
	// P = F*P*F' + Q
	transitionModel(X, U, dt); // X = f(X,U,dt)
	P = _F * P * _F.transpose() + _Q;
}

// performs EKF update, storing the result in X and P
// X: state vector that will be updated (n x 1)
// P: state covariance that will be updated (n x n)
// Z: observation (s x 1)
// dt: change in time
// TODO: for speed, the transpose of H can be computed once per model.
void RbpfModel::update(VectorNd &X, MatrixNNd &P, const VectorSd &Z, double dt) {
	// Yhat = Z - h(Xhat), h() = observation model
	// S = H*P*H' + R
	// K = P*H'*S^{-1}
	// Xhat = Xhat + K*Yhat
	// P = (I - K*H)*P
	observationModel(X,_h); // h = h(Xhat)
	_Yhat = Z-_h;
	_S = _H*P*_H.transpose() + _R;

	MatrixNSd K = P * _H.transpose() * _S.inverse();
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
	out << "F:\n" << model._F << std::endl;
	out << "H:\n" << model._H << std::endl;
	out << "Q:\n" << model._Q << std::endl;
	out << "R:\n" << model._R << std::endl;
	return out;
}
