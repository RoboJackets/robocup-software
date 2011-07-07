#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class DynamicNavigationTC: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				DynamicNavigationTC(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			private:

        ObstaclePtr centerObstacle[2];
        ObstaclePtr quadrantObstacle[4];
        ObstaclePtr goalObstacle[2];

				static ConfigDouble *_opp_avoid_radius;
				static ConfigDouble *_goal_X;
				static ConfigDouble *_goal_Y;
				static ConfigDouble *_center_X;
		};
	}
}
