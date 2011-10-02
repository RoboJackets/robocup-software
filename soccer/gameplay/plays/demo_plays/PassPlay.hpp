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
				static void createConfiguration(Configuration *cfg);
				PassPlay(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
			protected:
				Behaviors::Kick _passer1;
				Behaviors::Kick _passer2;
				bool _passer1HasBall;
				uint64_t readyTime;

				static ConfigBool *_use_chipper;
				static ConfigBool *_use_line;
				static ConfigInt *_kick_power;
				static ConfigInt *_backup_time;
				static ConfigDouble *_backup_speed;
		};
	}
}
