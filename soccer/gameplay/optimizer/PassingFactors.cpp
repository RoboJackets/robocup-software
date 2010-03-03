/*
 * @file PassingFactors.cpp
 * @author Alex Cunningham
 */

#include <boost/bind.hpp>
#include "PassingFactors.hpp"

using namespace gtsam;
using namespace std;
using namespace Gameplay;
using namespace Optimization;


Gameplay::Optimization::OpponentPassAvoidFactor::OpponentPassAvoidFactor(
		const SelfKey& kicker, const SelfKey& receiver,
		const OppKey& opp, double sigma)
 : NonlinearFactor<Config>(noiseModel::Isotropic::Sigma(1, sigma)),
   passer_(kicker), receiver_(receiver), opp_(opp)
   {
   }

/** Vector of errors */
Vector
Gameplay::Optimization::OpponentPassAvoidFactor::unwhitenedError(const Config& c) const {
	const Self_t& kicker = c[passer_];
	const Self_t& receiver = c[receiver_];
	const Opp_t&  opp = c[opp_];

	// cost is maximizing distance squared to the midpoint of the line
	// TODO: switch to actual distance to the line after figuring out the derivatives

	// Note that we negate the cost because we want a big distance
	return Vector_(1, -1*pointSegmentDist(kicker.t(), receiver.t(), opp));
}

/** linearize to a GaussianFactor */
boost::shared_ptr<gtsam::GaussianFactor>
Gameplay::Optimization::OpponentPassAvoidFactor::linearize(const Config& c) const {
	const Self_t& kicker = c[passer_];
	const Self_t& receiver = c[receiver_];
	const Opp_t&  opp = c[opp_];

	cout << "In OpponentPassAvoidFactor::linearize" << endl;

	// Derivative matrices
	Matrix Ak, Ar, Ao;

	Vector b = Vector_(1, -pointSegmentDist(kicker.t(), receiver.t(), opp,
											Ak, Ar, Ao));

	gtsam::print(Ak, "Ak");

	// bake in the noise information
	this->noiseModel_->WhitenInPlace(Ak);
	this->noiseModel_->WhitenInPlace(Ar);
	this->noiseModel_->WhitenInPlace(Ao);
	this->noiseModel_->whitenInPlace(b);

	// signs flipped so that this maximizes distance rather than minimizes it
	return GaussianFactor::shared_ptr(new GaussianFactor(
			passer_, -Ak, receiver_, -Ar, opp_, -Ao, b,
			noiseModel::Unit::Create(1)));
}

