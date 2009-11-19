/*
 * RbpfModel.hpp
 *
 *  General overview:
 *  A particle model forms a node in the model graph that handles the 'linear'
 *  portions of the filtering. This is implemented as an Extended Kalman Filter
 *  (EKF) and this model contains everything from an EKF except for the state
 *  and covariance themselves, which are maintained within the particle filter.
 *
 *  Implementation details:
 *
 *  Note:
 *    For ease of understanding, when possible the notation here is based on:
 *      http://en.wikipedia.org/wiki/Kalman_filter
 *      http://en.wikipedia.org/wiki/Particle_filter
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#ifndef RBPFMODEL_HPP_
#define RBPFMODEL_HPP_

#include <iostream>
#include <fstream>
#include <assert.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "InvertMatrix.hpp"
using std::ostream;
using std::endl;
namespace ublas = boost::numeric::ublas;
typedef ublas::vector<double> Vector;
typedef ublas::matrix<double> Matrix;

// Constants for state sizes across all models
// If these are changed, make sure to modify the virtual functions, such as
// transitionModel(), which are written assuming specific state sizes
#define NSIZE 6 // size of state
#define MSIZE 6 // size of control input
#define SSIZE 2 // size of measurement

class RbpfModel {
public:
	// Constructor: RbpfModel()
	RbpfModel();

	// Destructor: ~RbpfModel()
	~RbpfModel();

	// Operator: <<
	//   Used for printing this model to a stream (debugging)
	friend ostream& operator<<(ostream& out, const RbpfModel &model);

	// Function: predict(&X, &P, &U, dt)
	//   performs EKF predict, storing the result in X and P
	//   X: state vector that will be updated (n x 1)
	//   P: state covariance that will be updated (n x n)
	//   U: control input (m x 1)
	//   dt: change in time
	void predict(Vector &X, Matrix &P, Vector &U, double dt);

	// Function: update(&X, &P, &Z, dt)
	//   performs EKF update, storing the result in X and P
	//   X: state vector that will be updated (n x 1)
	//   P: state covariance that will be updated (n x n)
	//   Z: observation (s x 1)
	//   dt: change in time
	void update(Vector &X, Matrix &P, Vector &Z, double dt);

	// Function: getPredictedMeasurement()
	//   returns the previously predicted measurement, h (s x 1)
	Vector* getPredictedMeasurement();

	// Function: getInnovation()
	//   returns the previously calculated innovation, Yhat (s x 1)
	Vector* getInnovation();

	// Function: getInnovationCovariance()
	//   returns the previously calculated innovation, S (s x s)
	Matrix* getInnovationCovariance();

	// public variables
	const int n;    // size of state
	const int m;    // size of control input
	const int s;    // size of measurement

protected:
	// Function: transitionModel(&X,&U)
	//   X: state vector that will be updated (n x 1)
	//   U: control input (m x 1)
	//   dt: change in time
	virtual void transitionModel(Vector &X, Vector &U, double dt) = 0;

	// Function: computeTransitionJacobian(dt)
	//   Computes the transition Jacobian and stores the result in F
	//   Must call before predict()
	virtual void computeTransitionJacobian(double dt) = 0;

	// Function: observationModel(&X, &out)
	//   X: state vector (n x 1)
	//   out: observation (s x 1)
	virtual void observationModel(Vector &X, Vector &out) = 0;

	// Function: computeObservationJacobian(dt)
	//   computes the Jacobian of the observation Model function, wrt the state
	//   and stores the result in H.
	//   Must call before update()
	virtual void computeObservationJacobian(double dt) = 0;


	// protected variables
	Matrix F; // state transition Jacobian (df/dx) (n x n)
	Matrix H; // observation Jacobian (dh/dx) (s x n)
	Matrix Q; // process noise (n x n)
	Matrix R; // measurement noise (s x s)

	// variables used in intermediate calculations
	Matrix Inn;  // identity matrix (n x n)
	Vector h;    // predicted measurement (s x 1)
	Vector Yhat; // innovation (s x 1)
	Matrix S;    // innovation (or residual) covariance (s x s)

};

#endif /* RBPFMODEL_HPP_ */
