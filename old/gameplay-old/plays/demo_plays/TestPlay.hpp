#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TestPlay: public Play
		{
			public:
				TestPlay(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
		};
	}
}
