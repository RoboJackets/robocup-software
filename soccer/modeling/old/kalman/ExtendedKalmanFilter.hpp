/**
 * ExtendedKalmanFilter.hpp
 *
 * Very simple EKF implementation for position and velocity estimation
 *
 *  Created on: Jul 22, 2010
 *      Author: Philip Rogers
 */
#pragma once

#include <LinearAlgebra.hpp>

namespace Modeling
{
	class ExtendedKalmanFilter {
	public:

		// Q: 4x4 process covariance, R: 2x2 observation covariance
		// These control the uncertainty on the update (e.g., friction on
		// ground) and measurement (e.g., camera error.)
		ExtendedKalmanFilter(const LinAlg::Matrix &Q, const LinAlg::Matrix &R);
		virtual ~ExtendedKalmanFilter();

		// update the filter estimate to some other time (aka coast the filter)
		void predict(double dt);

		// update the filter, given a new observation
		void update(LinAlg::Vector &Z);

		// predict and update the filter given a new observation dt time from
		// the previous call of update
		void update(LinAlg::Vector &Z, double dt);

		LinAlg::Vector _X; // state vector (4 x 1) {x, y, vx, vy}

	private:
		LinAlg::Matrix _P; // state covariance (4 x 4)

		void computeTransitionJacobian(double dt);
		void applyTransitionModel(double dt);
		void computeObservationJacobian();
		void applyObservationModel();

		LinAlg::Matrix _F;    // state transition Jacobian (df/dx) (4 x 4)
		LinAlg::Matrix _H;    // observation Jacobian (dh/dx) (2 x 4)
		LinAlg::Matrix _Q;    // process noise (4 x 4)
		LinAlg::Matrix _R;    // measurement noise (2 x 2)
		LinAlg::Matrix _Inn;  // identity matrix (4 x 4)
		LinAlg::Vector _h;    // predicted measurement (2 x 1)
		LinAlg::Vector _Yhat; // innovation (2 x 1)
		LinAlg::Matrix _S;    // innovation (or residual) covariance (2 x 2)
	};
}

