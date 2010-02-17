/*
 * @file PassingFactors.hpp
 * @brief Factors used for modeling passing
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/BetweenFactor.h>
#include <gtsam/Vector.h>
#include "passOptimization.hpp"

namespace Gameplay {
	namespace Optimization {

	/** Pass shortening factor
	 * Pulls start and end together
	 */
	class PassShorteningFactor : public gtsam::BetweenFactor<Config, SelfKey, Self_t> {
		public:
			typedef gtsam::BetweenFactor<Config, SelfKey, Self_t> Base;
			PassShorteningFactor(const SelfKey& start, const SelfKey& end, double sigma)
				: Base(start, end, gtsam::Pose2(), gtsam::noiseModel::Isotropic::Sigma(3, sigma)) {}
	};

	} // \Optimization
} // \Gameplay
