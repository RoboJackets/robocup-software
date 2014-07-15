#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/positions/Fullback.hpp>

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
				Behaviors::Fullback _fullbackLeft;
				Behaviors::Fullback _fullbackRight;
		};
	}
}
