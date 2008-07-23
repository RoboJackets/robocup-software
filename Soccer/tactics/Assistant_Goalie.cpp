#include "Assistant_Goalie.hpp"
#include "../Role.hpp"

#include <Geometry/Point2d.hpp>

using namespace Geometry;

Tactics::Factory_Type<Tactics::Assistant_Goalie> assistant_goalie("assistant_goalie");

Tactics::Assistant_Goalie::Assistant_Goalie(Role *role) :
    Base(role)
{
}

float Tactics::Assistant_Goalie::score(Robot* r)
{
	//gimped robot fixes
	return r->id();
}

void Tactics::Assistant_Goalie::run()
{
	//controlled by the goalie
	
	//TODO flag for controlled
	
	//if not controlled, needs to do something useful
	
	if (robot()->free())
	{
		robot()->move(Point2d(0, 1.0));
	}
}
