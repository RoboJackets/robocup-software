#include "Behavior.hpp"

#include <boost/foreach.hpp>

Gameplay::Behavior::Behavior(GameplayModule *gameplay)
{
	_gameplay = gameplay;
}

Gameplay::Behavior::~Behavior()
{
}

bool Gameplay::Behavior::run()
{
	return false;
}

bool Gameplay::Behavior::allVisible() const
{
	if (_robots.empty())
	{
		return false;
	}
	
	BOOST_FOREACH(Robot *r, _robots)
	{
		if (!r->visible())
		{
			return false;
		}
	}
	
	return true;
}

void Gameplay::Behavior::assign(std::set<Robot *> &available)
{
	takeBest(available);
}

void Gameplay::Behavior::takeAll(std::set<Robot *> &available)
{
	_robots = available;
	available.clear();
}

Gameplay::Robot *Gameplay::Behavior::takeBest(std::set<Robot *> &available)
{
	if (available.empty())
	{
		return 0;
	}
	
	float bestScore = 0;
	Robot *best = 0;
	BOOST_FOREACH(Robot *r, available)
	{
		if (!best)
		{
			best = r;
		} else {
			float s = score(r);
			if (s < bestScore)
			{
				bestScore = s;
				best = r;
			}
		}
	}
	
	// best is guaranteed not to be null because we already ensured that available is not empty.
	
	available.erase(best);
	_robots.insert(best);
	
	return best;
}

float Gameplay::Behavior::score(Robot *r)
{
	return 0;
}
