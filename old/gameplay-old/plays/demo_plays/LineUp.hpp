#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class LineUp: public Play
		{
			public:
				LineUp(GameplayModule *gameplay);

				static float score(GameplayModule *gameplay);
				virtual bool run();
		};
	}
}
