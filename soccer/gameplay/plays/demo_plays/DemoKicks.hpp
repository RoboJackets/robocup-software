#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class DemoKicks: public Play
		{
			public:
				typedef enum {
					Deg0StationaryKick,
					Deg45StationaryKick,
					NumDemos // make sure this is last
				} Demo;

				DemoKicks(GameplayModule *gameplay);

				/** returns true if the play is currently applicable given gamestate */
				virtual bool applicable();

				/** Assigns robots to the play given a set of robots */
				virtual bool assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:
				// Insert sub behaviors here as member variables
				Demo _test;

				// flag for controlling switching between plays
				bool switchDemo;

				// current robot
				Robot* robot;

				// returns the robot in _robots with id id
				Robot* getRobotWithId(int id);
		};
	}
}
