/**
 *  Play takes all robots and executes a pre-computed PassConfig
 */

#include <iostream>
#include <boost/foreach.hpp>
#include "TestPassConfigOptimize.hpp"
#include "../../optimizer/PassOptimizer.hpp"

using namespace std;
using namespace Geometry2d;
using namespace Packet;
using namespace Optimization;

Gameplay::Plays::TestPassConfigOptimize::TestPassConfigOptimize(GameplayModule *gameplay):
	Play(gameplay),
	testState_(INIT),
	optimizer_(gameplay)
{
}

bool Gameplay::Plays::TestPassConfigOptimize::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

void Gameplay::Plays::TestPassConfigOptimize::assign(set<Robot *> &available)
{
	takeAll(available);
}

bool Gameplay::Plays::TestPassConfigOptimize::run()
{
	bool verbose = true;

	if (testState_ == INIT)
	{
		// create the config
		const Geometry2d::Point initBallPos = ball().pos;
		Geometry2d::Point goalBallPos = Geometry2d::Point(0.0, Constants::Field::Length);

		// create the config and initialize the first step
		PassConfig passConfig;
		passConfig.addPassState(PassState(initBallPos,PassState::INITIAL)); // add initial ball pos

		// create the intermediate steps
		BOOST_FOREACH(Robot * r, _robots)
		{
			if (r->visible()) {
				Geometry2d::Point final_robot_pos = r->pos() + (goalBallPos - r->pos()).normalized()*0.15;
				Geometry2d::Point ballPos = final_robot_pos + (goalBallPos - r->pos()).normalized()*0.15;
				passConfig.addPassState(PassState(ballPos, r, final_robot_pos));     // add intercept pos
			}
		}

		// create the final shot
		passConfig.addPassState(PassState(goalBallPos,PassState::GOAL));// add goal state

		// save to the secondary config in gameplay for drawing
		config_ = passConfig;
		gameplay()->_passConfig_secondary = &config_; // this is a persistent object

		// print for debugging:
		if (verbose) cout << "Initial Config" << passConfig << endl;

		// goto next state
		testState_ = OPTIMIZE;

	}
	else if (testState_ == OPTIMIZE)
	{
		// perform optimization
		opt_config_ = optimizer_.optimizePlan(config_, verbose);

		if (verbose) cout << "Optimized Config" << opt_config_ << endl;

		// save to primary config in gameplay for drawing
		gameplay()->_passConfig_primary = &opt_config_;

		// get out of state to avoid repeating calculations
		testState_ = DONE;
	}

	return true;
}

