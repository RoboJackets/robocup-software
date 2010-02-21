#pragma once

#include "../../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TestBallSpeed: public Play
		{
			public:
				TestBallSpeed(GameplayModule *gameplay);

				/** returns true if the play is currently applicable given gamestate */
				virtual bool applicable();

				/** Assigns robots to the play given a set of robots */
				virtual void assign(std::set<Robot *> &available);

				/** Called every frame */
				virtual bool run();

			protected:
				static const int Num_Speed_History = 3;
				Geometry2d::Point _last_pos;
				bool _first;
				float _speed_history[Num_Speed_History];
				float _max_speed;
				uint64_t _last_time;

				std::list<Geometry2d::Point> _pos_history;
		};
	}
}
