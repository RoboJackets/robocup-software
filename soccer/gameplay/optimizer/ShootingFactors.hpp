/*
 * @file ShootingFactors.hpp
 * @brief Factors for modeling shots
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/NonlinearFactor.h>
#include <gtsam/Vector.h>
#include "passOptimization.hpp"

namespace Gameplay {
	namespace Optimization {

	/** Shot shortening factor
	 * Makes shots on goal shorter
	 */
	class ShotShorteningFactor : public gtsam::NonlinearFactor1<Config, SelfKey, Self_t> {
		public:
			typedef gtsam::NonlinearFactor1<Config, SelfKey, Self_t> Base;
			ShotShorteningFactor(const SelfKey& pos, double sigma)
				: Base(gtsam::noiseModel::Isotropic::Sigma(6, sigma), pos) {}

			Vector evaluateError(const Self_t& pos, boost::optional<Matrix&> H1 = boost::none) const;
	};

	} // \Optimization
} // \Gameplay
