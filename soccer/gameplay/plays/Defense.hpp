#pragma once

#include <gameplay/Play.hpp>

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class Defense: public Play
		{
			public:
				Defense(GameplayModule *gameplay);
				
				virtual bool applicable(const std::set<Robot *> &robots);
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Forward _kicker1, _kicker2;
		};
	}
}
