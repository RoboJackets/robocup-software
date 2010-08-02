#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Move.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class DefendPenalty: public Play
		{
			public:
				DefendPenalty(GameplayModule *gameplay);
				
				virtual bool applicable(const std::set<Robot *> &robots);
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				Behaviors::Move _idle1, _idle2, _idle3, _idle4;
		};
	}
}
