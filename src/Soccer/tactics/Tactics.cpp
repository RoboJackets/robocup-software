#include "Tactics.hpp"
#include "../Role.hpp"

#include <stdexcept>
#include <boost/foreach.hpp>

#include <Geometry/Point2d.hpp>

using namespace Geometry;
using namespace std;

Tactics::Factory::~Factory()
{
}

Tactics::Base *Tactics::Factory::create(const std::string &name, Role *role)
{
    Factory *factory = find(name);
    if (factory)
    {
        return factory->create(role);
    } else {
        return 0;
    }
}

////////

Tactics::Base::Base(Role *role)
{
    _role = role;
    _robot = 0;

    if (_role)
    {
        _role->_tactics.push_back(this);
    }
}

Tactics::Base::~Base()
{
    if (_role)
    {
        // Remove from role
        vector<Tactics::Base *> &tactics = _role->_tactics;
        for (unsigned int i = 0; i < tactics.size(); ++i)
        {
            if (tactics[i] == this)
            {
                tactics.erase(tactics.begin() + i);
                break;
            }
        }
    }
}

float Tactics::Base::priority()
{
    return 0;
}

float Tactics::Base::score(Robot *robot)
{
    return 0;
}

Robot *Tactics::Base::select_robot()
{
    float best_score = 0;
    Robot *best_robot = 0;
    for (int i = 0; i < 5; ++i)
    {
        Robot *r = &Robot::self[i];
        if (r->visible() && !r->assigned())
        {
            float s = score(r);
            if (!best_robot || s < best_score)
            {
                best_score = s;
                best_robot = r;
            }
        }
    }

    return best_robot;
}

void Tactics::Base::robot(Robot *robot)
{
    // If we were running, stop.
    if (_robot)
    {
        stop();
    }

    _robot = robot;

    // If we are now running, start.
    if (_robot)
    {
        start();
    }
}

void Tactics::Base::start()
{
}

void Tactics::Base::stop()
{
}

void Tactics::Base::run()
{
	//IDLE position if nothing else runs
	robot()->move(Point2d(-2.22, .5 + .2 * robot()->id()));
	robot()->face(Point2d(0, 3.0));
}

bool Tactics::Base::done()
{
    return true;
}
