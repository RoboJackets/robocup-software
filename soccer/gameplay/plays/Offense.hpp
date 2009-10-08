#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class Offense: public Play
		{
			public:
				Offense(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual void assign(std::set<Robot *> &available);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Forward _kicker1, _kicker2;
		};
	}
}
