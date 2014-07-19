#pragma once

#include "../Play.hpp"
#include <gameplay/behaviors/Idle.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class Stopped: public Play
		{
			public:
				Stopped(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				Behaviors::Idle _idle;
				Behaviors::Defender _left, _right;
		};
	}
}
