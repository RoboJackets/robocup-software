/*
 * @file DrivingFactors.hpp
 * @brief Factors used to model driving constraints
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/BetweenFactor.h>
#include <gtsam/NonlinearFactor.h>
#include <gtsam/Vector.h>
#include "passOptimization.hpp"

namespace Gameplay {
	namespace Optimization {

	/** Path shortening factor
	 * Pulls variables closer together
	 */
	class PathShorteningFactor : public gtsam::BetweenFactor<Config, SelfKey, gtsam::Pose2> {
		public:
			typedef gtsam::BetweenFactor<Config, SelfKey, gtsam::Pose2> Base;
			PathShorteningFactor(const SelfKey& start, const SelfKey& end, double sigma)
				: Base(start, end, gtsam::Pose2(), gtsam::noiseModel::Isotropic::Sigma(3, sigma)) {}
	};

	/** factor for pivoting
	 * tries to make the angle between the poses small
	 */
	class ReaimFactor : public gtsam::BetweenFactor<Config, SelfKey, gtsam::Pose2> {
		public:
			typedef gtsam::BetweenFactor<Config, SelfKey, gtsam::Pose2> Base;
			ReaimFactor(const SelfKey& start, const SelfKey& end, double sigma)
				: Base(start, end, gtsam::Pose2(),
						gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector_(3, 0.5, 0.5, sigma))) {}
	};

	/** Factor to avoid being near opponents while driving */
	class OpponentAvoidanceFactor : public gtsam::NonlinearFactor2<Config, SelfKey, Self_t, OppKey, Opp_t> {
			public:
				typedef gtsam::NonlinearFactor2<Config, SelfKey, Self_t, OppKey, Opp_t> Base;

				OpponentAvoidanceFactor(const SelfKey& self, const OppKey& opp, double sigma);

				Vector
				evaluateError(const Self_t&, const Opp_t&, boost::optional<Matrix&> H1 =
						boost::none, boost::optional<Matrix&> H2 = boost::none) const;
	};

	} // \Optimization
} // \Gameplay

