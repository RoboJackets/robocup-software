/*
 * @file passOptimization.cpp
 * @author Alex Cunningham
 */

#include <iostream>
#include <gameplay/optimizer/passOptimization.hpp>

// implementations
#include <gtsam/NonlinearConstraint-inl.h>
#include <gtsam/NonlinearFactorGraph-inl.h>
#include <gtsam/TupleConfig-inl.h>
#include <gtsam/NonlinearOptimizer-inl.h>

using namespace Gameplay::Optimization;

// instantiations
namespace gtsam {
INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Gameplay::Optimization::Config)
INSTANTIATE_NONLINEAR_CONSTRAINT(Gameplay::Optimization::Config)
INSTANTIATE_TUPLE_CONFIG3(Gameplay::Optimization::OppConfig, Gameplay::Optimization::SelfConfig, Gameplay::Optimization::LagrangeConfig)
INSTANTIATE_NONLINEAR_OPTIMIZER(Gameplay::Optimization::Graph, Gameplay::Optimization::Config)
}

using namespace gtsam;
using namespace std;
using namespace Geometry2d;

Point2 Gameplay::Optimization::rc2gt_Point2(const Point& pt) {
	return Point2(pt.x, pt.y);
}

Point Gameplay::Optimization::gt2rc_Point2(const Point2& pt) {
	return Point(pt.x(), pt.y());
}

Pose2 Gameplay::Optimization::rc2gt_Pose2(const Point& pt, float angle) {
	return Pose2(pt.x, pt.y, angle);
}

pair<Point, float> Gameplay::Optimization::gt2rc_Pose2(const gtsam::Pose2& pt) {
	return make_pair(Point(pt.x(), pt.y()), pt.theta());
}

size_t Gameplay::Optimization::encodeID(uint8_t robotNum, size_t frame_num) {
	return frame_num + maxFrames*(robotNum+1);
}

uint8_t Gameplay::Optimization::decodeRobot(size_t id) {
	return id/maxFrames-1;
}

size_t Gameplay::Optimization::decodeFrame(size_t id) {
	return id % maxFrames;
}

double
Gameplay::Optimization::pointSegmentDist(
		const gtsam::Point2& start, const gtsam::Point2& end, const gtsam::Point2& pt,
		boost::optional<Matrix&> Dstart, boost::optional<Matrix&> Dend,
		boost::optional<Matrix&> Dpt)
{
	// distance squared to the midpoint of the line
	// TODO: switch to actual distance to the line after figuring out the derivatives
//	Point2 A = start, B = end;
//	Point2 AB = B-A;
//	double normalizer = AB.norm()/2.0;
//	Point2 midpoint(AB.x()*normalizer, AB.y()*normalizer);
//	double dist = midpoint.dist(pt);
//
//	if (!Dstart && !Dend && !Dpt) return dist*dist;
//
//	cout << "Calculating derivatives" << endl;
//	Vector dMdA = -0.5*A.vector()*AB.norm()-0.5*AB.vector();
//	Vector dMdB =  0.5*B.vector()*AB.norm()+0.5*AB.vector();
//
//	Point2 d1 = pt - midpoint; // mul by 2 for derivative
//
//	// derivatives
//	Matrix HA = Matrix_(1, 3, -d1.x()*dMdA(0), -d1.y()*dMdA(1), 0.0);
//	Matrix HB = Matrix_(1, 3, -d1.x()*dMdB(0), -d1.y()*dMdB(1), 0.0);
//	Matrix HC = Matrix_(1, 2, d1.x(), d1.y());
//
//	cout << "Calculated derivatives" << endl;
//	gtsam::print(HA, "Dstart in pointSegmentDist");
//
//	if (Dstart) *Dstart = HA;
//	if (Dend) *Dend = HB;
//	if (Dpt) *Dpt = HC;

	// Squared distance to line - Philip's derivation
	Point2 A = start, B = end, C = pt;
	Point2 AB = B-A, CA = A-C;
	double top = AB.x()*CA.y() - CA.x()*AB.y();
	double normalizer = dot(AB.vector(), AB.vector());
	double dist = top*top/normalizer;

	if (!Dstart && !Dend && !Dpt) return dist;

	double Dnormalizer = dot(-AB.vector(), -AB.vector());

	double Ap1 = (A.x()-B.x())*(B.x()-C.x()) + (A.y()-B.y())*(B.y()-C.y());
	double Ap2 = -B.y()*C.x() + A.y()*(C.x()-B.x()) + A.x()*(B.y()-C.y());

	double Bp1 = A.y()*(B.x()-C.x()) + B.y()*C.x() - B.x()*C.y() + A.x()*(C.y()-B.y());
	double Bp2 = dot(A.vector(), A.vector()) + B.x()*C.x() - A.x()*(B.x()+C.x()) + B.y()*C.y() - A.y()*(B.y() + C.y());

	double Cp = A.y()*(B.x()-C.x()) + B.y()*C.x() - B.x()*C.y() + A.x()*(C.y()-B.y());

	// derivatives
	Matrix HA = Matrix_(1, 3,
			-2*AB.y()*Ap1*Ap2/(Dnormalizer*Dnormalizer), // partial in x
			2*AB.y()*Ap1*Ap2/(Dnormalizer*Dnormalizer), // partial in y
			0.0);            // theta doesn't change
	Matrix HB = Matrix_(1, 3,
			-2*AB.y()*Bp1*Bp2/(Dnormalizer*Dnormalizer), // partial in x
			-2*AB.y()*Bp1*Bp2/(Dnormalizer*Dnormalizer), // partial in y
			0.0);            // theta doesn't change
	Matrix HC = Matrix_(1, 2,
			2*AB.y()*Cp/Dnormalizer,          // partial in x
			-2*AB.x()*Cp/Dnormalizer); 		 // partial in y

	if (Dstart) *Dstart = HA;
	if (Dend) *Dend = HB;
	if (Dpt) *Dpt = HC;


	return dist*dist;
}

