#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TestDefense: public Play
		{
			public:
				TestDefense(GameplayModule *gameplay);

				virtual bool run();

			protected:
				Behaviors::Defender _defenderLeft;
				Behaviors::Defender _defenderRight;
		};
	}
}
