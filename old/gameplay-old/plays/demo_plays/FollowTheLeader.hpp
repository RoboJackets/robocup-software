#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class FollowTheLeader: public Play
		{
			public:
				FollowTheLeader(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
		};
	}
}
