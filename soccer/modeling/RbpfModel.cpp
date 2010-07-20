/*
 * RbpfModel.cpp
 *
 *  See: RbpfModel.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include <modeling/RbpfModel.hpp>

// initializes n, m, s, F, H, Q, R, and space used for intermediates
// calculations: Inn (n x n Identity matrix), h, Yhat, S
RbpfModel::RbpfModel(Modeling::RobotModel::RobotMap *robotMap) : n(NSIZE), m(MSIZE), s(SSIZE), F(n,n), H(s,n), Q(n,n), R(s,s), _robotMap(robotMap), Inn(n,n), h(s), Yhat(s,1), S(s,s) {
	Inn.clear(); h.clear(); Yhat.clear(); S.clear(); // zero out matrices
	for(int i=0;i<n;i++){Inn(i,i)=1.0;} // initialize identity matrix
}

RbpfModel::~RbpfModel(){}

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
	assert((int)P.size1() == n); // P must be of size (n x n)
	assert((int)P.size2() == n); // P must be of size (n x n)
	assert((int)U.size() == m); // U must be of size (m x 1)
	// Xhat = f(X,U,dt), f() = state transition model
	// P = F*P*F' + Q
	computeTransitionJacobian(dt); // recompute F
	transitionModel(X, U, dt); // X = f(X,U,dt)
	Matrix FPFt = ublas::prod(F,P);
	FPFt = ublas::prod(FPFt,ublas::trans(F));
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
	assert((int)P.size1() == n); // P must be of size (n x n)
	assert((int)P.size2() == n); // P must be of size (n x n)
	assert((int)Z.size() == s); // Z must be of size (s x 1)
	// Yhat = Z - h(Xhat), h() = observation model
	// S = H*P*H' + R
	// K = P*H'*S^{-1}
	// Xhat = Xhat + K*Yhat
	// P = (I - K*H)*P
	computeObservationJacobian(dt); // recompute H
	observationModel(X,h); // h = h(Xhat)
	Yhat = Z-h;
	S = ublas::prod(H,P);
	S = ublas::prod(S,ublas::trans(H)) + R;
	Matrix K = ublas::prod(P,ublas::trans(H));
	Matrix Sinv(s,s);
	if(InvertMatrix(S,Sinv)){
		K = ublas::prod(K,Sinv);
		X = X + ublas::prod(K,Yhat);
		P = ublas::prod(Inn - ublas::prod(K,H),P);
	}else{
		// S^{-1} could not be determined, throw exception
		printf("S^-1 could not be calculated during EKF update in RbpfModel.update()");
		bool inverseFound = false;
		assert(inverseFound);
	}
}

// returns the previous predicted measurement, h (s x 1)
RbpfModel::Vector* RbpfModel::getPredictedMeasurement(){
	return &h;
}

// returns the previously calculated innovation, Y (s x 1)
RbpfModel::Vector* RbpfModel::getInnovation(){
	return &Yhat;
}
// returns the previously calculated innovation covariance, S (s x s)
RbpfModel::Matrix* RbpfModel::getInnovationCovariance(){
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
