#pragma once

#include <stdint.h>
#include "RobotModel.hpp"
#include "BallModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"

class Configuration;

namespace Modeling
{
	class ExtendedKalmanFilter;

	class EKFBallModel : public BallModel
	{
		public:
			EKFBallModel(RobotModel::RobotMap *robotMap, Configuration *config);
			virtual ~RBPFBallModel();

		protected:
			/** parameters for noise model */
			ConfigDouble _processNoiseSqrdPos;
			ConfigDouble _processNoiseSqrdVel;
			ConfigDouble _measurementNoiseSqrd;

			/** core filter implementation */
			ExtendedKalmanFilter * _ekf;

			/** implementation of the update function  - uses multiple observations */
			virtual void update(float dtime);

			/** simpler update for a single observation */
			void singleUpdate(float dtime);

			/** pull params from the config */
			void initParams();
	};
}
