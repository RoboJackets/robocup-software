/*
 * OptimizerGraph.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <cmath>
#include <boost/bind.hpp>
#include <gtsam/NonlinearEquality.h>
#include <gtsam/NonlinearConstraint.h>
#include <gtsam/NonlinearConstraint-inl.h>
#include <gameplay/optimizer/OptimizerGraph.hpp>
#include "OptimizerUtils.hpp"
#include "Factors.hpp"

using namespace Gameplay;
using namespace Optimization;
using namespace std;
using namespace gtsam;

typedef NonlinearEquality<OptimizerConfig> RC_nle;
typedef boost::shared_ptr<RC_nle> shared_nle;
typedef NonlinearConstraint1<OptimizerConfig> RC_nlc1;
typedef boost::shared_ptr<RC_nlc1> shared_nlc1;
typedef BasicUnaryFactor RC_nlf1;
typedef boost::shared_ptr<RC_nlf1> shared_nlf1;
typedef boost::shared_ptr<ShorteningFactor> shared_shortening;

OptimizerGraph::OptimizerGraph() {
	// TODO Auto-generated constructor stub

}

OptimizerGraph::~OptimizerGraph() {
	// TODO Auto-generated destructor stub
}

void OptimizerGraph::addBallIntercept(int robot_num, const Packet::LogFrame::Ball& ball) {
	// goal here is to get next behind the ball
	// predict where it will be a second
	double t = 1; // in seconds
	Geometry2d::Point target;
	if (ball.vel.mag() > 0.2) {
		// perform projection if the ball is moving
		target = ball.accel*0.5*t + ball.vel*t + ball.pos;
	} else {
		// just move to the ball if ball is stationary
		target = ball.pos;
	}

	// create the basic factor
	double weight = 0.1;
	cout << "Target for intercept: (" << target.x << ", " << target.y << ")" << endl;
	shared_nlf1 f1(new RC_nlf1(pt2vec(target), weight,
			Optimization::unary, genKey(robot_num, "final"),
			Optimization::Dunary));
	push_back(f1);
}

// general compare function for fixing variables
bool compare(const std::string& key, const OptimizerConfig& feasible, const OptimizerConfig& input) {
	Vector feas, lin;
	feas = feasible[key];
	lin = input[key];
	return equal_with_abs_tol(lin, feas, 1e-5);
}

void OptimizerGraph::addRobotInitConstraint(int robot_num, const Geometry2d::Point& pos) {
	// use a standard equality constraint here
	OptimizerConfig feasible;
	feasible.insert(genKey(robot_num, "init"), pt2vec(pos));
	size_t dim = 2;
	shared_nle init_constraint(new RC_nle(genKey(robot_num, "init"), feasible, dim, compare));
	push_back(init_constraint);
}

void OptimizerGraph::addRobotMotion(int robot_num, Robot* r) {
	// Basic: want to shorten the distance between the intial and final position
	double weight = 0.5;
	shared_shortening f1(new ShorteningFactor(weight, genKey(robot_num, "init"),
													  genKey(robot_num, "final")));
	push_back(f1);

	// TODO: avoid collisions between points
}

void OptimizerGraph::addRobotFieldBound(int robot_num) {
	// adds an inequality constraint to the final robot position to keep it on the field
	string lag_key = "L_" + genKey(robot_num, "final") + "_field";

	// create a constraint on X
	//FIXME: these constraints don't work right, and it's probably a systematic GTSAM problem
//	shared_nlc1 c1(new RC_nlc1(genKey(robot_num, "final"),
//			boost::bind(field_bound::grad_g, 0.0, 0, _1, _2),
//			boost::bind(field_bound::g_func, -0.5*Constants::Field::Width, 0.5*Constants::Field::Width, 0, _1, _1),
//			1, lag_key+"_x", false));
//
//	// create a constraint on Y
//	shared_nlc1 c2(new RC_nlc1(genKey(robot_num, "final"),
//			boost::bind(field_bound::grad_g, 0.5*Constants::Field::Length, 1, _1, _2),
//			boost::bind(field_bound::g_func, 0.0, Constants::Field::Length, 1, _1, _1),
//			1, lag_key+"_y", false));
//
//	// add the constraints
//	push_back(c1);
//	push_back(c2);
}

void OptimizerGraph::addPass(int passer, int receiver) {
	// add a basic path shortening factor to the pass
	double weight = 2.0;
	shared_shortening f1(new ShorteningFactor(weight, genKey(receiver, "final"), genKey(passer, "final")));
	push_back(f1);

	// TODO: avoid collisions between points
}

void OptimizerGraph::addShot(int robot_num) {
	// get the goal
	Geometry2d::Point goal(0.0, Constants::Field::Length);

	// add a weak basic unary factor to draw the shooter to the goal
	double weight = 2.0;
	shared_nlf1 f1(new RC_nlf1(pt2vec(goal), weight,
				Optimization::unary, genKey(robot_num, "final"),
				Optimization::Dunary));
	push_back(f1);

	// TODO: avoid collisions between points
}

void OptimizerGraph::addRobotPrior(int robot_num, double prior_weight) {
	//shared_nlf1 f(new RC_nlf1())
}

void OptimizerGraph::print(const std::string& name) const {
	NonlinearFactorGraph<OptimizerConfig>::print("OptimizerGraph: " + name);
}

bool OptimizerGraph::equals(const OptimizerGraph& expected, double tol) const {
	return false; //Placeholder
}

