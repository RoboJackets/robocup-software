#include "Behavior.hpp"
#include "Role.hpp"
#include "GameplayModule.hpp"

#include <stdexcept>
#include <boost/foreach.hpp>

using namespace Geometry2d;
using namespace std;

Gameplay::BehaviorFactory::~BehaviorFactory()
{
}

Gameplay::Behavior *Gameplay::BehaviorFactory::create(const std::string &name, GameplayModule *gameplay, Role *role)
{
    BehaviorFactory *factory = find(name);
    if (factory)
    {
        return factory->create(gameplay, role);
    } else {
        return 0;
    }
}

////////

Gameplay::Behavior::Behavior(GameplayModule *gameplay, Role *role)
{
    _gameplay = gameplay;
    _role = role;
    _robot = 0;

    if (_role)
    {
        _role->_behaviors.push_back(this);
    }
}

Gameplay::Behavior::~Behavior()
{
    if (_role)
    {
        // Remove from role
        vector<Behavior *> &behaviors = _role->_behaviors;
        for (unsigned int i = 0; i < behaviors.size(); ++i)
        {
            if (behaviors[i] == this)
            {
                behaviors.erase(behaviors.begin() + i);
                break;
            }
        }
    }
}

float Gameplay::Behavior::priority()
{
    return 0;
}

float Gameplay::Behavior::score(Robot *robot)
{
    return 0;
}

Gameplay::Robot *Gameplay::Behavior::selectRobot()
{
    float best_score = 0;
    Robot *best_robot = 0;
    for (int i = 0; i < 5; ++i)
    {
        Robot *r = gameplay()->self[i];
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

void Gameplay::Behavior::robot(Robot *robot)
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

void Gameplay::Behavior::start()
{
}

void Gameplay::Behavior::stop()
{
}

void Gameplay::Behavior::run()
{
}

bool Gameplay::Behavior::done()
{
    return true;
}
