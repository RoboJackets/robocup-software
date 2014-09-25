#pragma once

#include <stdint.h>
#include <memory>

//#define KALMANMODEL

#include <Geometry2d/Point.hpp>
#ifdef KALMANMODEL
#include <cblas.h>
#include "BLASWrap/blaswrap.h"
#include "kalman/difference_kalman.hpp"
#endif
#include <Configuration.hpp>

namespace Modeling
{
	class RobotModel
	{
		public:
			typedef std::shared_ptr<RobotModel> shared;
			typedef std::map<unsigned int, RobotModel::shared> RobotMap;

			// observations to be added from vision
			typedef struct {
				Geometry2d::Point pos;
				float angle;
				uint64_t time;
			} Observation_t;
			typedef std::vector<Observation_t> ObsVec;

			struct Config
			{
				Config(Configuration *config);
				
				ConfigDouble posAlpha;
				ConfigDouble posBeta;
				ConfigDouble posGamma;
				ConfigDouble angleAlpha;
				ConfigDouble angleBeta;
				ConfigDouble angleGamma;
			};
			
			RobotModel(Config *config, int s);

			void observation(uint64_t time, Geometry2d::Point pos, float angle);
			Geometry2d::Point predictPosAtTime(float dtime);
			void update(uint64_t cur_time);

			// set/get
			Geometry2d::Point pos() const { return _pos; }
			Geometry2d::Point vel() const { return _vel; }
			float angle() const { return _angle; }
			float angleVel() const { return _angleVel; }

			int shell() const { return _shell; }

			bool hasBall() const { return _haveBall; }
			void hasBall(bool a) { _haveBall = a; }

			/**
			 * Valid if there are new observations or if the robot has not timed out
			 */
			bool valid(uint64_t cur_time) const;

			uint64_t lastObservedTime;  /// The time used in the last observation, stored between frames
			uint64_t lastUpdatedTime;   /// The last time we performed an update on the filter

		private:
			unsigned int _shell;

			ObsVec _observations;

			// Maximum time to coast a track (keep the track alive with no observations) in microseconds.
			static const uint64_t MaxRobotCoastTime = 500000;

			// Current filtered state
			Geometry2d::Point _pos;
			Geometry2d::Point _vel;
			Geometry2d::Point _accel;
			float _angle;
			float _angleVel;
			float _angleAccel;

			Config *_config;

			// Data from RadioRx
			bool _haveBall;

#ifdef KALMANMODEL
			//Difference Kalman Filter
			DifferenceKalmanFilter *_posKalman;
			DifferenceKalmanFilter *_angKalman;

			/** Position **/
			//State Transition Matrix
			DMatrix posA;
			//Input Transition Matrix
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
			//State Transition Matrix
			DMatrix angA;
			//Input Transition Matrix
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
#endif
	};
}
