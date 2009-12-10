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
	if (testState_ == INIT)
	{
		// create the config
		const Geometry2d::Point initBallPos = ball().pos;
		Geometry2d::Point goalBallPos = Geometry2d::Point(0.0, Constants::Field::Length);

		// create the config and initialize the first step
		PassConfig passConfig;
		passConfig.addPassState(new PassState(initBallPos,PassState::INITIAL)); // add initial ball pos

		// create the intermediate steps
		BOOST_FOREACH(Robot * r, _robots)
		{
			if (r->visible()) {
				Geometry2d::Point final_robot_pos = r->pos() + (goalBallPos - r->pos()).normalized()*0.15;
				Geometry2d::Point ballPos = final_robot_pos + (goalBallPos - r->pos()).normalized()*0.15;
				passConfig.addPassState(new PassState(ballPos, r, final_robot_pos));     // add intercept pos
			}
		}

		// create the final shot
		passConfig.addPassState(new PassState(goalBallPos,PassState::GOAL));// add goal state

		// save to the secondary config in gameplay for drawing
		opt_config_ = passConfig;
		gameplay()->_passConfig_secondary = &opt_config_; // this is a persistent object

		// print for debugging:
		cout << passConfig << endl;

		// goto next state
		testState_ = OPTIMIZE;

	}
	else if (testState_ == OPTIMIZE)
	{
		// perform optimization


		// save to primary config in gameplay for drawing
	}

	return true;
}

