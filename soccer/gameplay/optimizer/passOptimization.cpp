/*
 * @file passOptimization.cpp
 * @author Alex Cunningham
 */

#include "passOptimization.hpp"

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
INSTANTIATE_TUPLE_CONFIG4(Gameplay::Optimization::OppConfig, Gameplay::Optimization::InitialConfig, Gameplay::Optimization::FinalConfig, Gameplay::Optimization::BallConfig)
INSTANTIATE_NONLINEAR_OPTIMIZER(Gameplay::Optimization::Graph, Gameplay::Optimization::Config)
}

using namespace gtsam;
using namespace std;
using namespace Geometry2d;

Point2 gt2rc_Point2(const Point& pt) {
	return Point2(pt.x, pt.y);
}

Point rc2gt_Point2(const Point2& pt) {
	return Point(pt.x(), pt.y());
}


