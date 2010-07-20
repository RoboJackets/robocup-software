/*
 * PassOptimizer.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Alex Cunningham
 */

#pragma once

#include <gameplay/GameplayModule.hpp>
#include <gameplay/optimizer/PassConfig.hpp>
#include <gameplay/optimizer/passOptimization.hpp>

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

			/** Generate a simple cost for a single PassConfig */
			double evaluateConfig(const PassConfig& cfg);

			/**
			 * Primary optimization function
			 * Takes a configuration and returns an optimized one
			 * with optional verbosity levels
			 */
			PassConfig optimizePlan(const PassConfig& init, bool verbose = false) const;

			/** parameters used in creation of a plan */
			// Make these smaller to make them more aggressive
			double fetchSigma;      /// determines how far to go while fetching
			double passRecSigma;    /// determines how far to go to receive pass
			double reaimSigma;      /// determines how far to while reaiming
			double shotLengthSigma; /// determines how much to shorten shots
			double passLengthSigma; /// determines how much to shorten shots
			double facingSigma;     /// Weight for pass facing factors
			double shotFacingSigma; /// Weight for shot facing factors
			double oppAvoidSigma;   /// Weight for opp avoidance during driving
			double oppPassAvoidSigma; /// Weight for opp avoidance during passes
			double oppShotAvoidSigma; /// Weight for opp avoidance during shots

			// Make this larger to make optimization more aggressive
			double priorSigma;      /// Weight for priors on all variables - bounds all optimization

		protected:
			/** the gameplay module link to get access to state information */
			GameplayModule * gameplay_;

			/** initializes a config based on a PassConfig */
			shared_config initializeConfig(const PassConfig& cfg) const;

			/** creates a factor graph */
			shared_graph createGraph(const PassConfig& cfg) const;
		};
	}
}

