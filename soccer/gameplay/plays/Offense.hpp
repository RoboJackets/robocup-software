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
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				bool _usingKicker1;

				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Kick _kicker1, _kicker2;
		};
	}
}
