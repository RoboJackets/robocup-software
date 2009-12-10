/*
 * PassOptimizer.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <boost/foreach.hpp>
#include <gameplay/optimizer/PassOptimizer.hpp>
#include "OptimizerGraph.hpp"
#include "OptimizerConfig.hpp"

using namespace std;

Gameplay::Optimization::PassOptimizer::PassOptimizer(GameplayModule* gameplay)
: gameplay_(gameplay)
{
}

Gameplay::Optimization::PassOptimizer::~PassOptimizer() {}

PassConfig Gameplay::Optimization::PassOptimizer::optimizePlan(
		const PassConfig& init, bool verbose) const
{
	// build up the initial configuration and graph by reading through PassConfig
	OptimizerConfig config;
	OptimizerGraph graph;

	// track the robots in order
	vector<Robot*> robotSequence;

	// loop through the pass states to add them to the graph
	BOOST_FOREACH(PassState s, init.passStateVector) {
		if (s.stateType == PassState::INITIAL) {

		} else if (s.stateType == PassState::INTERMEDIATE) {
			// save the robot to use
			robotSequence.push_back(s.robot);
		} else if (s.stateType == PassState::GOAL) {

		}
	}

	// add initial positions of the robots

	// fix the initial states

	// create an SQP optimizer to optimize the graph

	// iterate-solve using fixed iteration numbers for safety

	// if at end of iteration, the constraint-error is high, return the original plan

	// if solution is better, then convert the config back into a PassConfig and return
	return init;
}
