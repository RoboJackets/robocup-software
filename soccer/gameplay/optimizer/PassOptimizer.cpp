/*
 * PassOptimizer.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <boost/foreach.hpp>
#include <gtsam/Ordering.h>
#include <PassOptimizer.hpp>
#include <passOptimization.hpp>

using namespace std;
using namespace boost;
using namespace gtsam;
using namespace Gameplay;
using namespace Optimization;
using namespace Geometry2d;

Gameplay::Optimization::PassOptimizer::PassOptimizer(GameplayModule* gameplay)
: gameplay_(gameplay)
{
}

Gameplay::Optimization::PassOptimizer::~PassOptimizer() {}

PassConfig Gameplay::Optimization::PassOptimizer::optimizePlan(
		const PassConfig& init, bool verbose) const
{
//	// create graph and config
//	shared_config config(new Config());
//	shared_graph graph(new Graph());
//
//	// initialize current positions of our robots
//	uint8_t nrSelf = 0;
//	BOOST_FOREACH(Gameplay::Robot * robot, gameplay_->self) {
//		if (robot->visible()) {
//			Point pos = robot->pos();
//			graph->add(RobotSelfConstraint(nrSelf, 0, pos));
//			config->insert(SelfKey(encodeID(nrSelf, 0)), rc2gt_Point2(pos));
//		}
//		++nrSelf;
//	}
//
//	// initialize current position for opp robots
//	uint8_t nrOpp = 5;
//	BOOST_FOREACH(Gameplay::Robot * robot, gameplay_->opp) {
//		if (robot->visible()) {
//			Point pos = robot->pos();
//			graph->add(RobotOppConstraint(nrOpp, 0, pos));
//			config->insert(OppKey(encodeID(nrOpp, 0)), rc2gt_Point2(pos));
//		}
//		++nrOpp;
//	}
//
//	// initialize ball
//	Point ballPos = gameplay_->state()->ball.pos;
//	graph->add(BallConstraint(0, ballPos));
//	config->insert(BallKey(0), rc2gt_Point2(ballPos));
//
//
//	// go through initial passconfig and initialize the graph and the config
//	BOOST_FOREACH(PassState s, init.passStateVector) {
//		switch (s.stateType) {
//		case PassState::INTERMEDIATE :
//			break;
//		case PassState::KICKPASS :
//			break;
//		case PassState::RECEIVEPASS :
//			break;
//		case PassState::KICKGOAL :
//			break;
//		}
//	}
//
//	// optimize
//	shared_ptr<Ordering> ordering(new Ordering(graph->getOrdering()));
//	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
//	Optimizer optimizer(graph, config, solver);
//	double relThresh = 1e-3;
//	double absThresh = 1e-3;
//	size_t maxIt = 10;
//	Optimizer result = optimizer.levenbergMarquardt(relThresh, absThresh, Optimizer::SILENT, maxIt);

	// reconstruct a passconfig
	PassConfig optConfig(init);


	// OLD CODE: Saved for posterity
//	// build up the initial configuration and graph by reading through PassConfig
//	shared_config config(new OptimizerConfig);
//	OptimizerGraph graph;
//
//	// store the robots for lookup
//	vector<Robot*> robots;
//
//	// loop through the pass states to add them to the graph
//	int robot_num = 0;
//	PassState prevState;
//	BOOST_FOREACH(PassState s, init.passStateVector) {
//		if (s.stateType == PassState::INITIAL) {
//			// set up ball intercept for the first robot
//			graph.addBallIntercept(robot_num, gameplay_->state()->ball);
//
//		} else if (s.stateType == PassState::INTERMEDIATE) {
//			// pull out the robot and store
//			Robot* r = s.robot;
//			robots.push_back(r);
//
//			// add the robot to the config - initial and final poses
//			config->initRobot(robot_num, r->pos(), s.robotPos);
//			//config->initTime(robot_num, s); //TODO: add timing information
//
//			// add appropriate factors to the graph for basic movement
//			graph.addRobotInitConstraint(robot_num, r->pos()); /// fix initial point
//			graph.addRobotMotion(robot_num, r);				   /// add basic path shortening
//			//graph.addRobotFieldBound(robot_num);	           /// keep the robot on the field
//
////			// add some priors to the robot poses to avoid scatter
////			double prior_weight = 0.1;
////			graph.addRobotPrior(robot_num, prior_weight);
//
//			// if this robot is receiving a pass, create the pass factors
//			if (prevState.stateType == PassState::INTERMEDIATE) {
//				graph.addPass(robot_num, robot_num-1); // all-in-one pass creation from two points
//			}
//
//			// goto next robot
//			++robot_num;
//		} else if (s.stateType == PassState::GOAL) {
//			// create the shot factor
//			graph.addShot(robot_num-1);
//		}
//		// save the state for connections
//		prevState = s;
//	}
//
//	// get an ordering
//	Ordering ordering = graph.getOrdering();
//
//	// print out the parts
//	if (verbose) {
//		cout << "Printing initial graph and config..." << endl;
//		config->print("Initial");
//		graph.print("Initial");
//		ordering.print("Guessed Ordering for the graph");
//	}
//
//	// create an SQP optimizer to optimize the graph
//	Optimizer optimizer(graph, ordering, config);
//
//	if (verbose) {
//		optimizer.print("Initial Optimizer State");
//	}
//
//	// iterate-solve using fixed iteration numbers for safety
//	//TODO: do more than one iteration here, it'll be necessary
//	//Optimizer next = optimizer.iterate(Optimizer::SILENT);
//	double thresh = 1e-2;
//	int maxIt = 1;
//	Optimizer next = optimizer.iterateSolve(thresh, thresh, thresh, maxIt, Optimizer::SILENT);
//	shared_const_config newConfig = next.config();
//
//	if (verbose) {
//		newConfig->print("Optimized SQP Config");
//	}
//
//	// reconstruct the PassConfig from the updated configuration
//	PassConfig optConfig;
//	robot_num = 0;
//	BOOST_FOREACH(PassState s, init.passStateVector) {
//		PassState newState;
//		if (s.stateType == PassState::INITIAL) {
//			newState = PassState(s.ballPos, s.stateType);
//		} else if (s.stateType == PassState::INTERMEDIATE) {
//			newState = PassState(newConfig->getFinal(robot_num), s.robot, newConfig->getFinal(robot_num));
//			++robot_num;
//		} else if (s.stateType == PassState::GOAL) {
//			newState = PassState(s.ballPos, s.stateType);
//		}
//		optConfig.addPassState(newState);
//	}

	return optConfig;
}
