#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/Kick.hpp>
namespace Gameplay
{
	namespace Plays
	{
		class PassShootPlay: public Play
		{
			public:
				PassShootPlay(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
			protected:
				Behaviors::Kick _passer;
				Behaviors::Kick _kicker;
				bool _passerHasBall;
		};
	}
}
