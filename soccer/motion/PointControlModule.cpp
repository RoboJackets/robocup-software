/**
 * @file PointController.cpp
 * @author Alex Cunningham
 */

#include <boost/foreach.hpp>

#include "PointControlModule.hpp"

namespace Motion {

PointControlModule::PointControlModule(SystemState *state, Configuration *cfg)
: _state(state), _config(cfg)
{
	//initialize empty robots
	for(unsigned int i = 0; i < Num_Shells; i++)
		_robots[i] = RobotController::shared_ptr(new RobotController(_config, _state, i));
}

void PointControlModule::run()
{
	BOOST_FOREACH(RobotController::shared_ptr r, _robots)
		r->proc();
}

}
