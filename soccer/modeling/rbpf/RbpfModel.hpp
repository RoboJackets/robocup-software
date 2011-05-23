/*
 * RbpfModel.hpp
 *
 *  General overview:
 *  A particle model forms a node in the model graph that handles the 'linear'
 *  portions of the filtering. This is implemented as an Extended Kalman Filter
 *  (EKF) and this model contains everything from an EKF except for the state
 *  and covariance themselves, which are maintained within the particle filter.
 *
 *  Note:
 *    when possible the notation here is based on:
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
#include <LinearAlgebra.hpp>
#include "rbpfMatrices.h"
#include "../RobotModel.hpp"

class RbpfModel {
public:
	// robotMap is a set of robots for calculating kicks/deflections in some models
	RbpfModel(Modeling::RobotModel::RobotMap *robotMap);

	// Used for printing this model to a stream (debugging)
	friend std::ostream& operator<<(std::ostream& out, const RbpfModel &model);

	// Calculates the jacobians for this time step
	void computeJacobians(double dt) {
		computeObservationJacobian(dt);
		computeTransitionJacobian(dt);
	}

	// performs EKF predict, storing the result in X and P
	// X: state vector that will be updated (n x 1)
	// P: state covariance that will be updated (n x n)
	// U: control input (m x 1)
	// dt: change in time
	void predict(rbpf::VectorNd &X, rbpf::MatrixNNd &P, const rbpf::VectorMd &U, double dt) const;

	// performs EKF update, storing the result in X and P
	// X: state vector that will be updated (n x 1)
	// P: state covariance that will be updated (n x n)
	// Z: observation (s x 1)
	// dt: change in time
	// Updates Yhat and S
	virtual void update(rbpf::VectorNd &X, rbpf::MatrixNNd &P, const rbpf::VectorSd &Z, double dt);

	// functions that pull new values in from config files
	virtual void initializeQ()=0;
	virtual void initializeR()=0;

	// returns the previously predicted measurement, h (s x 1)
	rbpf::VectorSd& getPredictedMeasurement() { return _h; }

	// returns the previously calculated innovation, Yhat (s x 1)
	rbpf::VectorSd& getInnovation() { return _Yhat; }

	// returns the previously calculated innovation, S (s x s)
	rbpf::MatrixSSd& getInnovationCovariance() { return _S; }

	static const unsigned int n = NSIZE;    // size of state
	static const unsigned int m = MSIZE;    // size of control input
	static const unsigned int s = SSIZE;    // size of measurement

protected:
	// Function: transitionModel(&X,&U)
	//   X: state vector that will be updated (n x 1)
	//   U: control input (m x 1)
	//   dt: change in time
	virtual void transitionModel(rbpf::VectorNd &X, const rbpf::VectorMd &U, double dt) const = 0;

	// Computes the transition Jacobian and stores the result in F
	// Must call before predict()
	virtual void computeTransitionJacobian(double dt) = 0;

	// X: state vector (n x 1)
	// out: observation (s x 1)
	virtual void observationModel(const rbpf::VectorNd &X, rbpf::VectorSd &out) const = 0;

	// computes the Jacobian of the observation Model function, wrt the state
	// and stores the result in H.
	// Must call before update()
	virtual void computeObservationJacobian(double dt) = 0;

	rbpf::MatrixNNd _F; // state transition Jacobian (df/dx) (n x n)
	rbpf::MatrixSNd _H; // observation Jacobian (dh/dx) (s x n)
	rbpf::MatrixNNd _Q; // process noise (n x n)
	rbpf::MatrixSSd _R; // measurement noise (s x s)
	Modeling::RobotModel::RobotMap *_robotMap; // set of robots for kicks/deflections

	// variables used in intermediate calculations
	rbpf::MatrixNNd _Inn;  // identity matrix (n x n)
	rbpf::VectorSd  _h;    // predicted measurement (s x 1)
	rbpf::VectorSd  _Yhat; // innovation (s x 1)
	rbpf::MatrixSSd _S;    // innovation (or residual) covariance (s x s)
public:
	// Required for use of fixed size matrices as members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* RBPFMODEL_HPP_ */
