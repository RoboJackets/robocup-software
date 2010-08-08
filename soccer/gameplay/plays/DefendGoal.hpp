#pragma once

#include <gameplay/Play.hpp>

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>
#include <gameplay/behaviors/GoalDefender.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class DefendGoal: public Play
		{
			public:
				typedef enum
				{
					MarkAndDefend,
					DefendAndDefend,
					DefendAndMark
				} DefenseState;

				DefendGoal(GameplayModule *gameplay);

				static float score(GameplayModule *gameplay);
				virtual bool run();

			protected:
				DefenseState _defenseState;

				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Forward _kicker1, _kicker2;
				Behaviors::GoalDefender _goalDefender;
		};
	}
}
