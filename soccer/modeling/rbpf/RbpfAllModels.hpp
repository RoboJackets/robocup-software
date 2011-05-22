#if 0
/*
 * RbpfAllModels.hpp
 *
 *  Rather than spread these models across multiple files, I've jammed them all
 *  in here.  Each model is derived from the abstract RbpfModel class and
 *  simply implements the virtual methods defined there (RbpfModel.hpp).
 *
 *  Implementation details:
 *  Because all models require a constant state size, the state size for
 *  all models is defined at the top of RbpfModel.hpp, and must be constant
 *  across all models.  You may mix model sizes (i.e., 4D, 6D, etc.) in this
 *  file, but all the models instantiated at runtime must have sizes that
 *  correspond to those defined in RbpfModel.hpp.
 *
 *  This file contains class definitions of the following models:
 *    1.) RbpfModelRolling - This is a copy of Phillip Mark's Kalman Filter
 *                           (though this is the EKF variant).  Use a single
 *                           particle with this model to compare against the
 *                           original KF solution.
 *    2.) RbpfModelKicked - Rolling model with high acceleration noise
 *    3.) RbpfModelRollingFriction - model with rolling friction approximation
 *
 *  Author: Philip Rogers, Nov 19th 2009
 */

#ifndef RBPFALLMODELS_HPP_
#define RBPFALLMODELS_HPP_

#include <iostream>
#include <fstream>
#include <assert.h>
#include "RbpfModel.hpp"

// Class: RbpfModelRolling
//   free rolling ball, ignores control input
//   state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelRolling : public RbpfModel {
public:
	RbpfModelRolling();
	virtual ~RbpfModelRolling();
protected:
	void transitionModel(rbpf::VectorNf &X, rbpf::VectorMf &U, double dt);
	void computeTransitionJacobian(double dt);
	void observationModel(rbpf::VectorNf &X, rbpf::VectorSf &out);
	void computeObservationJacobian(double dt);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Class: RbpfModelKicked
//   kicked ball with high noise, ignores control input
//   Note: could be written to accept control input from our robots
//   state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelKicked : public RbpfModel {
public:
	RbpfModelKicked();
	virtual ~RbpfModelKicked();
protected:
	void transitionModel(rbpf::VectorNf &X, rbpf::VectorMf &U, double dt);
	void computeTransitionJacobian(double dt);
	void observationModel(rbpf::VectorNf &X, rbpf::VectorSf &out);
	void computeObservationJacobian(double dt);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Class: RbpfModelRollingFriction
//   rolling ball with friction, ignores control input
//   state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelRollingFriction : public RbpfModel {
public:
	RbpfModelRollingFriction();
	virtual ~RbpfModelRollingFriction();
protected:
	void transitionModel(rbpf::VectorNf &X, rbpf::VectorMf &U, double dt);
	void computeTransitionJacobian(double dt);
	void observationModel(rbpf::VectorNf &X, rbpf::VectorSf &out);
	void computeObservationJacobian(double dt);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* RBPFALLMODELS_HPP_ */
#endif
