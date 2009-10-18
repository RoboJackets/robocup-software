// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <stdint.h>

#include <Geometry2d/Point.hpp>
#include <cblas.h>
#include "BLASWrap/blaswrap.h"
#include "difference_kalman.hpp"

namespace Modeling
{
	class BallModel
	{
		public:
			BallModel();

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
	};
}
