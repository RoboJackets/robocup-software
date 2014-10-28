#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/tactics/passing/PassReceive.hpp>
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
				Behaviors::PassReceive _pass;
				bool _passer1HasBall;
				bool positiveX;

				static ConfigBool *_use_chipper;
				static ConfigBool *_use_line;
				static ConfigInt *_kick_power;
		};
	}
}
