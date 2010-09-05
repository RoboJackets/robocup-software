/**
 * @file PointController.cpp
 * @author Alex Cunningham
 */

#include <boost/foreach.hpp>

#include "PointController.hpp"

namespace Motion {

PointController::PointController(SystemState *state, Configuration *cfg)
: _state(state), _config(cfg)
{
	//initialize empty robots
    for(unsigned int i = 0; i < Num_Shells; i++)
    {
        _robots[i] = Robot::shared_ptr(new Robot(_config, _state, i));
	}
}

void PointController::run()
{
    BOOST_FOREACH(Robot::shared_ptr r, _robots)
    {
		r->proc();
    }
}

}
