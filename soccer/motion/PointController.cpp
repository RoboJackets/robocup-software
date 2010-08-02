/**
 * @file PointController.cpp
 * @author Alex Cunningham
 */

#include <boost/foreach.hpp>

#include "PointController.hpp"

namespace Motion {

PointController::PointController(SystemState *state, const ConfigFile::MotionModule& cfg)
: _state(state), _config(cfg)
{
	//initialize empty robots
    for(unsigned int i = 0; i < Constants::Robots_Per_Team; i++)
    {
        _robots[i] = Robot::shared_ptr(new Robot(cfg.robot, i));
		_robots[i]->setSystemState(_state);
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
