#pragma once

#include "../Play.hpp"

#include "../behaviors/Kick.hpp"
#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class ClearBall: public Play
		{
		public:
			ClearBall(GameplayModule *gameplay);

			static float score(GameplayModule *gameplay);
			
			virtual bool run();

		protected:

			virtual float scoreRobot(Robot *r);

			Behaviors::Kick _kicker;
			Behaviors::Fullback _fullback1;
			Behaviors::Forward _kicker1, _kicker2;
			bool _done;
		};
	} // \Gameplay
} // \Plays
