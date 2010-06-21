#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		/** Prevents a target robot from receiving a pass */
		class Mark: public Behavior
		{
			public:

				Mark(GameplayModule *gameplay);

				/** set the robot to mark */
				void markRobot(Robot * robot) { _markRobot = robot; }
				Robot * markRobot() const { return _markRobot;}

				/** set the ratio */
				void ratio(float r);
				float ratio() const { return _ratio; }

				virtual bool run();

			protected:
				// ratio for where to try to block - on zero to one
				// zero is stay at ball, one is at robot
				float _ratio;

				// the target
				Robot * _markRobot;
		};
	}
}
