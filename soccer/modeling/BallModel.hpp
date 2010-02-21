// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <stdint.h>

#include <Geometry2d/Point.hpp>
#include <cblas.h>
#include "BLASWrap/blaswrap.h"
#include "difference_kalman.hpp"

/* RBPF Includes */
#include <iostream>
#include <fstream>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "Rbpf.hpp"
#include "RbpfState.hpp"
#include "RbpfModel.hpp"
#include "RbpfAllModels.hpp"

typedef Geometry2d::Point Point;
typedef boost::numeric::ublas::vector<double> Vector;
typedef boost::numeric::ublas::matrix<double> Matrix;

namespace Modeling
{
	class BallModel
	{
		public:
			typedef enum {
				MODELTESTS,
				RBPF,
				KALMAN,
				ABG
			} mode_t;

			BallModel(mode_t mode = RBPF);

			void observation(uint64_t time, const Geometry2d::Point &pos);

			Geometry2d::Point predictPosAtTime(float dtime);
			void update();

			// Best observation so far
			Geometry2d::Point observedPos;
			uint64_t bestObservedTime;
			float bestError;

			/** Kalman **/
			//State Transistion Matrix
			DMatrix A;
			//Input Transistion Matrix
			DMatrix B;
			//Initial Covariance Matrix
			DMatrix P;
			//Process Covariance Matrix
			DMatrix Q;
			//Measurement Covariance Matrix
			DMatrix R;
			//Measurement Model
			DMatrix H;
			//Measurement
			DVector Z;
			//Input
			DVector U;
			//Initial Condition
			DVector X0;

			//Difference Kalman Filter
			DifferenceKalmanFilter *posKalman;

			/** Alpha Beta Gamma **/
			// Current filtered state
			Geometry2d::Point pos;
			Geometry2d::Point abgPos;
			Geometry2d::Point vel;
			Geometry2d::Point accel;

			// Filter coefficients
			float alpha;
			float beta;
			float gamma;

			uint64_t lastObservedTime;
			int missedFrames;

			Geometry2d::Point prevObservedPos;

			// filter errors
			float kalmanTestPosError;
			float rbpfTestPosError;
			float kalmanTestVelError;
			float rbpfTestVelError;

		protected:
			// mode of the filter
			mode_t mode_;

			// new particle filter implementation
			Rbpf* raoBlackwellizedParticleFilter;

			// Initialization functions for each mode

			/**
			 * Initialize Rao-Blackwellized Particle Filter
			 *   Constructs initial state X, initial covariance P, adds several models
			 *   to the modelGraph, and sets some transition weights
			 */
			void initRBPF();

			/** Initialize the older implementation of the ball filter */
			void initKalman();

			/** Initializes the Alpha-Beta-Gamma filter */
			void initABG();

			// update functions

			/** Old-style Kalman update */
			void kalmanUpdate(float dtime);

			/** Newer RBPF update */
			void rbpfUpdate(float dtime);

			/** ABG Filter update */
			void abgUpdate(float dtime);

	};
}
