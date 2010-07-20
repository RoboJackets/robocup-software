/*
 * @file ShootingFactors.hpp
 * @brief Factors for modeling shots
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/NonlinearFactor.h>
#include <gtsam/Vector.h>
#include <gameplay/optimizer/passOptimization.hpp>

namespace Gameplay {
	namespace Optimization {

	/** Shot shortening factor
	 * Makes shots on goal shorter
	 */
	class ShotShorteningFactor : public gtsam::NonlinearFactor1<Config, SelfKey, Self_t> {
		public:
			typedef gtsam::NonlinearFactor1<Config, SelfKey, Self_t> Base;
			ShotShorteningFactor(const SelfKey& pos, double sigma)
				: Base(gtsam::noiseModel::Isotropic::Sigma(3, sigma), pos) {}

			Vector evaluateError(const Self_t& pos, boost::optional<Matrix&> H1 = boost::none) const;
	};

	// a factor for aiming properly at the goal
	class ShootFacingFactor: public gtsam::NonlinearFactor1<Config, SelfKey, Self_t> {
	private:

		typedef gtsam::NonlinearFactor1<Config, SelfKey, Self_t> Base;

	public:

		ShootFacingFactor(); /* Default constructor */
		ShootFacingFactor(const SelfKey& i, const gtsam::SharedGaussian& noiseModel) :
			Base(noiseModel, i) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Self_t& pose, boost::optional<Matrix&> H1) const {
			gtsam::Point2 goal(0.0f, Constants::Field::Length);
			gtsam::Rot2 hx = bearing(pose, goal, H1, boost::none);
			return logmap(between(gtsam::Rot2(), hx));
		}
	}; // BearingFactor

	/** Avoid shooting through opponents */
	class OpponentShotAvoidanceFactor: public gtsam::NonlinearFactor2<Config, SelfKey, Self_t, OppKey, Opp_t> {
	private:

		typedef gtsam::NonlinearFactor2<Config, SelfKey, Self_t, OppKey, Opp_t> Base;

	public:

		OpponentShotAvoidanceFactor(); /* Default constructor */
		OpponentShotAvoidanceFactor(const SelfKey& self, const OppKey& opp, double sigma);

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Self_t& pose,  const Opp_t& opp,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const;

	}; // OpponentShotAvoidanceFactor


	} // \Optimization
} // \Gameplay
