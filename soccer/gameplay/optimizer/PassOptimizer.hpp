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

		protected:
			/** the gameplay module link to get access to state information */
			GameplayModule * gameplay_;
		};
	}
}

