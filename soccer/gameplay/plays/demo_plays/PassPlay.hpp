#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/Kick.hpp>
namespace Gameplay
{
	namespace Plays
	{
		class PassPlay: public Play
		{
			public:
				PassPlay(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
			protected:
				Behaviors::Kick _passer1;
				Behaviors::Kick _passer2;
				bool _passer1HasBall;
		};
	}
}
