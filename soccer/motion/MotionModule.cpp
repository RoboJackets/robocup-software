// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <motion/MotionModule.hpp>

#include <Constants.hpp>
#include <Configuration.hpp>

#include <QMouseEvent>
#include <boost/foreach.hpp>

using namespace std;
using namespace Motion;
using namespace Packet;

MotionModule::MotionModule(SystemState *state, Configuration *config)
{
	_state = state;

	//initialize empty robots
    for(int i = 0; i < Constants::Robots_Per_Team; i++)
    {
        _robots[i] = new Robot(config, _state, i);
	}
}

MotionModule::~MotionModule()
{
    for (int i = 0; i < Constants::Robots_Per_Team; i++)
    {
        delete _robots[i];
        _robots[i] = 0;
    }
}

void MotionModule::run()
{
    BOOST_FOREACH(Robot* r, _robots)
    {
		r->proc();
    }
}
