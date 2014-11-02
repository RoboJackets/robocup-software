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
				static void createConfiguration(Configuration *cfg);

				PassShootPlay(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();
			protected:
				Behaviors::Kick _passer;
				Behaviors::Kick _kicker;
				bool _passerHasBall;

				static ConfigBool *_pass_use_chipper;
				static ConfigBool *_pass_use_line;
				static ConfigBool *_shoot_use_chipper;
				static ConfigBool *_shoot_use_line;
				static ConfigInt *_pass_kick_power;
				static ConfigInt *_shoot_kick_power;
		};
	}
}
