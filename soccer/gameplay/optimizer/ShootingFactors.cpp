/*
 * @file ShootingFactors.cpp
 * @author Alex Cunningham
 */

#include <Constants.hpp>
#include <gameplay/optimizer/ShootingFactors.hpp>

using namespace gtsam;
using namespace std;
using namespace Gameplay;
using namespace Optimization;

Vector Gameplay::Optimization::ShotShorteningFactor::evaluateError(
		const Self_t& pos, boost::optional<Matrix&> H1) const {
	// Cost function is just a distance to the center of the goal using just translation
	Point2 goalCenter(0.0, Constants::Field::Length);
	Point2 posT = pos.t();
	Vector ret = (posT - goalCenter).vector();

	if (H1) *H1 = eye(3);
	return Vector_(3, ret(0), ret(1), 0.0);
}

Gameplay::Optimization::OpponentShotAvoidanceFactor::OpponentShotAvoidanceFactor(
		const SelfKey& self, const OppKey& opp, double sigma)
	: Base(noiseModel::Isotropic::Sigma(1, sigma), self, opp)
{
}

Vector
Gameplay::Optimization::OpponentShotAvoidanceFactor::evaluateError(
		const Self_t& pose,  const Opp_t& opp,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const
{
	Point2 goalCenter(0.0, Constants::Field::Length);
	Point2 posT = pose.t();

	Matrix Ak, Ao;
	Vector dist = Vector_(1, -pointSegmentDist(pose.t(), goalCenter, opp,
											   Ak, boost::none, Ao));

	if (H1) *H1 = -Ak;
	if (H2) *H2 = -Ao;
	return -dist;
}

