// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <Geometry2d/Point.hpp>
#include <framework/ConfigFile.hpp>
#include <cblas.h>
#include "BLASWrap/blaswrap.h"
#include "difference_kalman.hpp"

namespace Modeling
{
	class RobotModel
	{
		public:
			typedef boost::shared_ptr<RobotModel> shared;
			typedef std::map<int, shared> RobotMap;

			RobotModel(const ConfigFile::WorldModel& cfg, int s);

			void observation(uint64_t time, Geometry2d::Point pos, float angle);
			Geometry2d::Point predictPosAtTime(float dtime);
			void update();

			int shell;

			// Best observation so far
			Geometry2d::Point observedPos;
			uint64_t bestObservedTime;
			float observedAngle;
			float bestError;
			/** Position **/
			//State Transistion Matrix
			DMatrix posA;
			//Input Transistion Matrix
			DMatrix posB;
			//Initial Covariance Matrix
			DMatrix posP;
			//Process Covariance Matrix
			DMatrix posQ;
			//Measurement Covariance Matrix
			DMatrix posR;
			//Measurement Model
			DMatrix posH;
			//Measurement
			DVector posZ;
			//Input
			DVector posU;
			//Error
			DVector posE;
			//Initial Condition
			DVector posX0;

			/** Angle **/
			//State Transistion Matrix
			DMatrix angA;
			//Input Transistion Matrix
			DMatrix angB;
			//Initial Covariance Matrix
			DMatrix angP;
			//Process Covariance Matrix
			DMatrix angQ;
			//Measurement Covariance Matrix
			DMatrix angR;
			//Measurement Model
			DMatrix angH;
			//Measurement
			DVector angZ;
			//Input
			DVector angU;
			//Error
			DVector angE;
			//Initial Condition
			DVector angX0;

			//Difference Kalman Filter
			DifferenceKalmanFilter *posKalman;
			DifferenceKalmanFilter *angKalman;

			//Alpha Gamma Beta Filter
			// Current filtered state
			Geometry2d::Point pos;
			Geometry2d::Point abgPos;
			Geometry2d::Point vel;
			Geometry2d::Point accel;
			float angle;
			float abgAngle;
			float angleVel;
			float angleAccel;

			// Filter coefficients
			float posAlpha;
			float posBeta;
			float posGamma;
			float angleAlpha;
			float angleBeta;
			float angleGamma;

			uint64_t firstObservedTime;
			uint64_t lastObservedTime;

			/**
			 * When robot is no longer visible, disable, but remember the
			 * last position
			 */
			void deactivate();

			bool inUse;   /// currently used as a robot on the field
			bool isValid; /// currently receiving updates

			const ConfigFile::WorldModel& _config;
	};
}
