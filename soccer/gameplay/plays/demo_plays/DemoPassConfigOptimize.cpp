/**
 *  Play takes all robots and executes a pre-computed PassConfig
 */

#include <iostream>
#include <boost/foreach.hpp>
#include "DemoPassConfigOptimize.hpp"
#include <gameplay/optimizer/PassOptimizer.hpp>

using namespace std;
using namespace Geometry2d;
using namespace Packet;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoPassConfigOptimize, "Demos")

Gameplay::Plays::DemoPassConfigOptimize::DemoPassConfigOptimize(GameplayModule *gameplay):
	Play(gameplay),
	testState_(INIT),
	analyticPlanner_(gameplay),
	optimizer_(gameplay)
{
}

bool Gameplay::Plays::DemoPassConfigOptimize::run()
{
	if (testState_ == INIT)
	{
		// get the plans
		AnalyticPassPlanner::PassConfigVector initialPlans;
		analyticPlanner_.generateAllConfigs(ball().pos,_robots,initialPlans);
		analyticPlanner_.evaluateConfigs(_robots,_gameplay->opp,initialPlans);

		if (initialPlans.size() == 0)
			throw runtime_error("No plans created!");

		// copy out first plan and optimize
		config_ = initialPlans[0];

		double initError = optimizer_.evaluateConfig(config_);
		cout << "executing optimization - started with cost = " << initError << endl;
		opt_config_ = optimizer_.optimizePlan(config_, true);
		cout << "  optimization complete! - final cost = " << optimizer_.evaluateConfig(opt_config_) << endl;

		// go to next state
		testState_ = DemoPassConfigOptimize::SHOW;

	}
	else if (testState_ == SHOW)
	{
		QColor optC = Qt::cyan;
		QColor initC = Qt::darkCyan;
		config_.drawConfig(gameplay()->state(), initC.red(), initC.green(), initC.blue());
		opt_config_.drawConfig(gameplay()->state(), optC.red(), optC.green(), optC.blue());
	}

	return true;
}

