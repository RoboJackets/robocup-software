#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class Offense: public Play
		{
			public:
				Offense(GameplayModule *gameplay);
				
				virtual bool applicable(const std::set<Robot *> &robots);
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
				
			protected:

				bool _usingKicker1;

				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Kick _kicker1, _kicker2;
		};
	}
}
