#include "Behavior.hpp"

#include <boost/foreach.hpp>

using namespace std;

Gameplay::Behavior::Behavior(GameplayModule *gameplay)
	: _gameplay(gameplay)
{
}

Gameplay::Behavior::~Behavior()
{
}

void Gameplay::Behavior::unassign()
{
	set<Robot *> empty;
	assign(empty);
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
		if (r && !r->visible())
		{
			return false;
		}
	}

	return true;
}

bool Gameplay::Behavior::assign(std::set<Robot *> &available)
{
	_robots.clear();
	return takeBest(available);
}

bool Gameplay::Behavior::takeAll(std::set<Robot *> &available)
{
	_robots = available;
	available.clear();
	return true;
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
		float s = score(r);
		if (!best)
		{
			bestScore = s;
			best = r;
		} else {
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
