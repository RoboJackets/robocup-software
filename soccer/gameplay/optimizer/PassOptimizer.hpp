/*
 * PassOptimizer.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Alex Cunningham
 */

#pragma once

#include <GameplayModule.hpp>
#include <PassConfig.hpp>

namespace Gameplay
{
	namespace Optimization
	{
		class PassOptimizer {

		public:
			/**
			 * Default constructor needs access to the gameplay for state information
			 */
			PassOptimizer(GameplayModule* gameplay);

			virtual ~PassOptimizer();

			/**
			 * Primary optimization function
			 * Takes a configuration and returns an optimized one
			 * with optional verbosity levels
			 */
			PassConfig optimizePlan(const PassConfig& init, bool verbose = false) const;

			/** parameters used in creation of a plan */
			double fetchSigma;   /// determines how far to go while fetching
			double passRecSigma; /// determines how far to go to receive pass
			double reaimSigma;   /// determines how far to while reaiming
			double shotLengthSigma; /// determines how much to shorten shots
			double passLengthSigma; /// determines how much to shorten shots

		protected:
			/** the gameplay module link to get access to state information */
			GameplayModule * gameplay_;
		};
	}
}

