#pragma once

#include "../Play.hpp"
#include <gameplay/behaviors/Idle.hpp>
#include <gameplay/behaviors/positions/Fullback.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class Stopped: public Play
		{
			public:
				Stopped(GameplayModule *gameplay);
				
				virtual bool applicable(const std::set<Robot *> &robots);
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				Behaviors::Idle _idle;
				Behaviors::Fullback _left, _right;
		};
	}
}
