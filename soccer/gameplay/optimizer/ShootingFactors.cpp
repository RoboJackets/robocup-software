/*
 * @file ShootingFactors.cpp
 * @author Alex Cunningham
 */

#include <common/Constants.hpp>
#include "ShootingFactors.hpp"

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
