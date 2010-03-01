/*
 * @file DrivingFactors.cpp
 * @author Alex Cunningham
 */

#include "DrivingFactors.hpp"

using namespace gtsam;
using namespace std;
using namespace Gameplay;
using namespace Optimization;

Gameplay::Optimization::OpponentAvoidanceFactor::OpponentAvoidanceFactor(
		const SelfKey& self, const OppKey& opp, double sigma)
	: Base(gtsam::noiseModel::Isotropic::Sigma(1, sigma), self, opp)
{
}

Vector
Gameplay::Optimization::OpponentAvoidanceFactor::evaluateError(
		const Self_t& self, const Opp_t& opp,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

	Matrix Aself, Aopp;
	double cost = -gtsam::range(self, opp, Aself, Aopp);
	if (H1) *H1 = -Aself;
	if (H2) *H2 = -Aopp;
	return Vector_(1, cost); // negated to maximize rather than minimize
}

