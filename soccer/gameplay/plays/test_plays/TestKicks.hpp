/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TestKicks: public Play
		{
			public:
				typedef enum {
					Deg0StationaryKick,
					Deg45StationaryKick,
					NumTests // make sure this is last
				} Test;

				TestKicks(GameplayModule *gameplay);

				/** returns true if the play is currently applicable given gamestate */
				virtual bool applicable();

				/** Assigns robots to the play given a set of robots */
				virtual bool assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:
				// Insert sub behaviors here as member variables
				Test _test;

				// flag for controlling switching between plays
				bool switchTest;

				// current robot
				Robot* robot;

				// returns the robot in _robots with id id
				Robot* getRobotWithId(int id);
		};
	}
}
