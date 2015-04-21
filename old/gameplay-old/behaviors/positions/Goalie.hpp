
#pragma once

#include "../../Behavior.hpp"
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	class WindowEvaluator;
	
	namespace Behaviors
	{
		class Goalie: public SingleRobotBehavior
		{
			public:
				static void createConfiguration(Configuration *cfg);
				Goalie(GameplayModule *gameplay);
				virtual ~Goalie();

				// Takes one robot out of <available> and makes it the goalie.
				// This will be called every frame.
				void assign(std::set<OurRobot *> &available, int ID);
				
				virtual bool run();

			protected:
				typedef enum
				{
					Defend,
					Block,
					Intercept,
					Clear,
					SetupPenalty,
					None
				} State;
				
				WindowEvaluator* _win;
				
				Kick _kick;
				
				State _state, _previousState;
				
				unsigned int _index;
			private:
				bool opponentsHavePossession();
				Robot* opponentWithBall();
				bool ballIsMovingTowardsGoal();
		};
	}
}
