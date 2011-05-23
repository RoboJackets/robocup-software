/*
 * RbpfState.hpp
 *
 *  State for a particle in a RBPF.  Maintains state and covariance of a
 *  Kalman Filter, a weight in the particle filter, and a pointer to the
 *  process model and process noise (determined by which mode the particle
 *  is in.)
 *
 *  Note:
 *    when possible the notation here is based on:
 *      http://en.wikipedia.org/wiki/Kalman_filter
 *      http://en.wikipedia.org/wiki/Particle_filter
 *
 *  Author: Philip Rogers, Nov 8th 2009
 */

#pragma once

#include <iostream>
#include <LinearAlgebra.hpp>
#include "rbpfMatrices.h"

class RbpfState {
public:

	// _X: initial state, (n x 1)
	// _P: initial state covariance, (n x n)
	// _modelIdx: index of this particle's model
	// _w: initial weight
	//  n: size of Kalman Filter state
	RbpfState(const rbpf::VectorNd& _X, const rbpf::MatrixNNd& _P, int _modelIdx, double _w);

	// Copies one state into another. Note: this is not a copy constructor.
	void copy(const RbpfState &state)
	{
		*this = state;
	}

	// Used for printing this state to a stream
	friend std::ostream& operator<<(std::ostream& out, const RbpfState &state);

	static const unsigned int n = NSIZE;         // size of Kalman Filter state
	rbpf::VectorNd X;      // state vector (n x 1)
	rbpf::MatrixNNd P;      // state covariance (n x n)
	int modelIdx;  // index of this particle's model
	double weight; // particle weight

	// Required for use of fixed size matrices as members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
