/*
 * RbpfModel.cpp
 *
 *  See: RbpfModel.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include "RbpfModel.hpp"

using namespace LinAlg;

// initializes n, m, s, F, H, Q, R, and space used for intermediates
// calculations: Inn (n x n Identity matrix), h, Yhat, S
RbpfModel::RbpfModel(Modeling::RobotModel::RobotMap *robotMap)
: n(NSIZE), m(MSIZE), s(SSIZE), F(n,n), H(s,n),
  Q(n,n), R(s,s), _robotMap(robotMap), Inn(Matrix::Identity(n,n)),
  h(Vector::Zero(s)), Yhat(Vector(s)), S(Matrix::Zero(s,s)) {
}

// performs EKF predict, storing the result in X and P
// X: state vector that will be updated (n x 1)
// P: state covariance that will be updated (n x n)
// U: control input (m x 1)
// dt: change in time
// TODO: for speed, computeTransitionJacobian can be performed once per model.
// TODO: for speed, the transpose of F can be computed once per model.
// TODO: for speed, temp matrices should be removed.
void RbpfModel::predict(Vector &X, Matrix &P, Vector &U, double dt){
	assert((int)X.size() == n);  // X must be of size (n x 1)
	assert((int)P.rows() == n); // P must be of size (n x n)
	assert((int)P.cols() == n); // P must be of size (n x n)
	assert((int)U.size() == m); // U must be of size (m x 1)
	// Xhat = f(X,U,dt), f() = state transition model
	// P = F*P*F' + Q
	computeTransitionJacobian(dt); // recompute F
	transitionModel(X, U, dt); // X = f(X,U,dt)
	Matrix FPFt = F * P;
	FPFt *= F.transpose();
	P = FPFt + Q;
}

// performs EKF update, storing the result in X and P
// X: state vector that will be updated (n x 1)
// P: state covariance that will be updated (n x n)
// Z: observation (s x 1)
// dt: change in time
// TODO: for speed, the transpose of H can be computed once per model.
// TODO: for speed, temp matrices should be removed.
void RbpfModel::update(Vector &X, Matrix &P, Vector &Z, double dt){
	assert((int)X.size() == n);  // X must be of size (n x 1)
	assert((int)P.rows() == n); // P must be of size (n x n)
	assert((int)P.cols() == n); // P must be of size (n x n)
	assert((int)Z.size() == s); // Z must be of size (s x 1)
	// Yhat = Z - h(Xhat), h() = observation model
	// S = H*P*H' + R
	// K = P*H'*S^{-1}
	// Xhat = Xhat + K*Yhat
	// P = (I - K*H)*P
	computeObservationJacobian(dt); // recompute H
	observationModel(X,h); // h = h(Xhat)
	Yhat = Z-h;
	S = H*P*H.transpose() + R;

	Matrix K = P * H.transpose() * S.inverse();
	X += K * Yhat;
	P = (Inn - K * H) * P;

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

// returns the previous predicted measurement, h (s x 1)
Vector* RbpfModel::getPredictedMeasurement(){
	return &h;
}

// returns the previously calculated innovation, Y (s x 1)
Vector* RbpfModel::getInnovation(){
	return &Yhat;
}
// returns the previously calculated innovation covariance, S (s x s)
Matrix* RbpfModel::getInnovationCovariance(){
	return &S;
}

// overloaded operator for printing a model (debugging)
// prints: F, H, Q, and R
std::ostream& operator<<(std::ostream& out, const RbpfModel &model){
	out << "F:" << std::endl;
	for(int x=0; x<model.n; x++){
		out << "\t";
		for(int y=0; y<model.n; y++){out << model.F(x,y) << ", ";}
		out << std::endl;
	}
	out << std::endl << "H:" << std::endl;
	for(int x=0; x<model.s; x++){
		out << "\t";
		for(int y=0; y<model.n; y++){out << model.H(x,y) << ", ";}
		out << std::endl;
	}
	out << std::endl << "Q:" << std::endl;
	for(int x=0; x<model.n; x++){
		out << "\t";
		for(int y=0; y<model.n; y++){out << model.Q(x,y) << ", ";}
		out << std::endl;
	}
	out << std::endl << "R:" << std::endl;
	for(int x=0; x<model.s; x++){
		out << "\t";
		for(int y=0; y<model.s; y++){out << model.R(x,y) << ", ";}
		out << std::endl;
	}
	return out;
}
