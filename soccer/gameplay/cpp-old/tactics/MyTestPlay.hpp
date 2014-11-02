#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Mark.hpp>
#include <gameplay/behaviors/Move.hpp>

using namespace std;
using namespace Geometry2d;

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * Offense with one main striker and a supporting robot that marks nearby robots
		 * Acts as 1-1-2 field play (one forward, one midfield, two defenders)
		 */
		class MyTestPlay: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				MyTestPlay(GameplayModule *gameplay);

				static float score(GameplayModule *gameplay);
				virtual bool run();

			private:
				int _letter;
				uint64_t _startTime;
				void getToPoint(Geometry2d::Point p, Behaviors::Move b, set<OurRobot *> &robots);
				void resetMoves();
				void waitFor(int secs);

			protected:
				Behaviors::Defender _leftDefender, _rightDefender;
				Behaviors::Move _move1;
				Behaviors::Move _move2;
				Behaviors::Move _move3;
				Behaviors::Move _move4;
				Behaviors::Move _move5;
				Behaviors::Kick _striker;
				Behaviors::Mark _support1, _support2;

				static ConfigDouble *_offense_hysteresis;  // determines when to switch offense players
				static ConfigDouble *_support_backoff_thresh;
				static ConfigDouble *_mark_hysteresis_coeff;
				static ConfigDouble *_support_avoid_teammate_radius;
				static ConfigDouble *_support_avoid_shot;
				static ConfigDouble *_offense_support_ratio;
				static ConfigDouble *_defense_support_ratio;
				static ConfigBool   *_use_line_kick;
		};
	}
}
