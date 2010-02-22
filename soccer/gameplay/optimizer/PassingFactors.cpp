/*
 * @file PassingFactors.cpp
 * @author Alex Cunningham
 */

#include <boost/bind.hpp>
//#include <gtsam/BearingFactor.h>
#include <gtsam/NonlinearConstraint-inl.h>
#include "PassingFactors.hpp"

using namespace gtsam;
using namespace std;
using namespace Gameplay;
using namespace Optimization;



//NonlinearConstraint2(
//		Vector (*g)(const Config& config),
//		const Key1& key1,
//		Matrix (*G1)(const Config& config),
//		const Key2& key2,
//		Matrix (*G2)(const Config& config),
//		size_t dim_constraint,
//		const LagrangeKey& lagrange_key,
//		bool isEquality=true);

/**
 * Calculate bearing and optional derivative(s)
 */
//Rot2 bearing(const Pose2& pose, const Point2& point,
//		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
//	if (!H1 && !H2) return bearing(pose, point);
//	Point2 d = transform_to(pose, point);
//	Matrix D_result_d;
//	Rot2 result = relativeBearing(d, D_result_d);
//	if (H1) *H1 = D_result_d * Dtransform_to1(pose, point);
//	if (H2) *H2 = D_result_d * Dtransform_to2(pose, point);
//	return result;
//}

///*
// * Bidirectional facing term - one robot tries to face the other, want the bearing to
// * the other robot to be zero so that the robot exactly faces the other target
// */
//Vector facing_g(const Config& config, const SelfKey& key1, const SelfKey& key2) {
//	// extract poses
//	Pose2 p1 = config[key1], p2 = config[key2];
//
//	// compute bearings in each direction
//	Rot2 h12 = bearing(p1, p2.t()),
//		 h21 = bearing(p2, p1.t());
//
//	return Vector_(2, logmap(between(Rot2(), h12))(0), logmap(between(Rot2(), h12))(0));
//}
//
//Matrix facing_G1(const Config& config, const SelfKey& key1, const SelfKey& key2) {
//	// extract poses
//	Pose2 p1 = config[key1], p2 = config[key2];
//
//	// compute bearings and derivatives
//	Matrix h12_1, h12_2, h21_1, h21_2;
//	Rot2 h12 = bearing(p1, p2.t(), h12_1, h12_2),
//		 h21 = bearing(p2, p1.t(), h21_1, h21_2);
//
//	// assemble the gradient matrix for key1
//	Matrix G1 = zeros(2,3);
//	insertSub(G1, h12_1, 0, 0);
//	insertSub(G1, h21_2, 1, 0);
//	return G1;
//}
//
//Matrix facing_G2(const Config& config, const SelfKey& key1, const SelfKey& key2) {
//	// extract poses
//	Pose2 p1 = config[key1], p2 = config[key2];
//
//	// compute bearings and derivatives
//	Matrix h12_1, h12_2, h21_1, h21_2;
//	Rot2 h12 = bearing(p1, p2.t(), h12_1, h12_2),
//		 h21 = bearing(p2, p1.t(), h21_1, h21_2);
//
//	// assemble the gradient matrix for key2
//	Matrix G2 = zeros(2,3);
//	insertSub(G2, h12_2, 0, 0);
//	insertSub(G2, h21_1, 1, 0);
//	return G2;
//}
//
//
///* Note that this is a dim=4 constraint, rotation for two robots */
//PassFacingConstraint::PassFacingConstraint(const SelfKey& key1, const SelfKey& key2,
//										   const LagrangeKey& lamKey)
//: Base(boost::bind(facing_g, _1, key1, key2),
//		key1, boost::bind(facing_G1, _1, key1, key2),
//		key2, boost::bind(facing_G2, _1, key1, key2),
//		2, lamKey)
//  {
//  }
