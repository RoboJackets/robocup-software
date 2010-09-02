#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class Follow: public Play
		{
			public:
				Follow(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
		};
	}
}
