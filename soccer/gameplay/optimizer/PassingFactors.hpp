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

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement,
	 * i.e. the main building block for visual SLAM.
	 */
	template<class Config, class PoseKey, class PointKey>
	class PoseBearingFactor: public gtsam::NonlinearFactor2<Config, PoseKey, gtsam::Pose2,
			PointKey, gtsam::Pose2> {
	private:

		gtsam::Rot2 z_; /** measurement */

		typedef gtsam::NonlinearFactor2<Config, PoseKey, gtsam::Pose2, PointKey, gtsam::Pose2> Base;

	public:

		PoseBearingFactor(); /* Default constructor */
		PoseBearingFactor(const gtsam::Rot2& z, const gtsam::SharedGaussian& noiseModel, const PoseKey& i,
				const PointKey& j) :
			Base(noiseModel, i, j), z_(z) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const gtsam::Pose2& pose, const gtsam::Pose2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			gtsam::Rot2 hx = bearing(pose, point.t(), H1, H2);
			return logmap(between(z_, hx));
		}
	}; // BearingFactor


	/** Factor for facing between poses */
	class PassFacingFactor : public PoseBearingFactor<Config, SelfKey, SelfKey> {
		public:
			PassFacingFactor(const SelfKey& near, const SelfKey& far, const gtsam::SharedGaussian& noiseModel) :
				PoseBearingFactor<Config, SelfKey, SelfKey>(gtsam::Rot2(), noiseModel, near, far) {}
	};

	/** Pass shortening factor
	 * Pulls start and end together
	 */
	class PassShorteningFactor : public gtsam::BetweenFactor<Config, SelfKey, Self_t> {
		public:
			typedef gtsam::BetweenFactor<Config, SelfKey, Self_t> Base;
			PassShorteningFactor(const SelfKey& start, const SelfKey& end, double sigma)
				: Base(start, end, gtsam::Pose2(), gtsam::noiseModel::Isotropic::Sigma(3, sigma)) {}
	};

	/**
	 * Factor that maximizes distance between the opp and the pass trajectory
	 */
	class OpponentPassAvoidFactor : public gtsam::NonlinearFactor<Config> {
		protected:
			/** Keys */
			SelfKey passer_, receiver_;
			OppKey opp_;

		public:
			OpponentPassAvoidFactor(const SelfKey& kicker, const SelfKey& receiver,
					const OppKey& opp, double sigma);

			/** Vector of errors */
			Vector unwhitenedError(const Config& c) const;

			/** linearize to a GaussianFactor */
			boost::shared_ptr<gtsam::GaussianFactor>
			linearize(const Config& c) const;
	};

//	/**
//	 * Takes two poses and makes them face each other
//	 */
//	class PassFacingConstraint : public gtsam::NonlinearConstraint2<Config, SelfKey, Self_t, SelfKey, Self_t> {
//		public:
//			typedef gtsam::NonlinearConstraint2<Config, SelfKey, Self_t, SelfKey, Self_t> Base;
//			PassFacingConstraint(const SelfKey& key1, const SelfKey& key2,
//								 const gtsam::LagrangeKey& lamKey);
//	};

	} // \Optimization
} // \Gameplay
