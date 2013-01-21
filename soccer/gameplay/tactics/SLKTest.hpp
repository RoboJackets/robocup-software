#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/tactics/StableLineKick.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class SLKTest: public Play
		{
			public:
				SLKTest(GameplayModule *gameplay);

				virtual bool run();

			protected:
				StableLineKick _kicker;
		};
	}
}
