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

        std::shared_ptr<Obstacle> centerObstacle[2];
        std::shared_ptr<Obstacle> quadrantObstacle[4];
        std::shared_ptr<Obstacle> goalObstacle[2];

				static ConfigDouble *_opp_avoid_radius;
				static ConfigDouble *_goal_X;
				static ConfigDouble *_goal_Y;
				static ConfigDouble *_center_X;
		};
	}
}
