#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Move.hpp>
#include <gameplay/behaviors/Penalty.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class KickPenalty: public Play
		{
			public:
				KickPenalty(GameplayModule *gameplay);
				
				virtual bool applicable(const std::set<Robot *> &robots);
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				Behaviors::Penalty _kicker;
				Behaviors::Move _idle1, _idle2, _idle3;
		};
	}
}
