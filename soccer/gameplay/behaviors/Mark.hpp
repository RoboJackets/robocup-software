#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		/** Prevents a target robot from receiving a pass */
		class Mark: public SingleRobotBehavior
		{
			public:

				Mark(GameplayModule *gameplay);

				/** set the robot to mark */
				void markRobot(OpponentRobot * robot) { _markRobot = robot; }
				OpponentRobot * markRobot() const { return _markRobot;}

				/** set the ratio */
				void ratio(float r);
				float ratio() const { return _ratio; }

				/** set the threshold */
				void markLineThresh(float thresh) { _mark_line_thresh = thresh; }
				float markLineThresh() const { return _mark_line_thresh; }

				virtual bool run();

			protected:
				// ratio for where to try to block - on zero to one
				// zero is stay at ball, one is at robot
				float _ratio;

				// distance threshold for determining how close this
				// robot needs to be to the mark line before
				// considering the ratio position
				float _mark_line_thresh;

				// the target
				OpponentRobot * _markRobot;
		};
	}
}
