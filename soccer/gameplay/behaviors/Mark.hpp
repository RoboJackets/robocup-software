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

				/** set the radius */
				void radius(float r) { _radius = r; }
				float radius() const { return _radius; }

				virtual bool run();

			protected:
				// the target
				Robot * _markRobot;

				float _radius; /// preferred radius to stay at to mark
		};
	}
}
