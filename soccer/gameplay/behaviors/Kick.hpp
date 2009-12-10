#pragma once

#include "../Behavior.hpp"

#include "Intercept.hpp"
#include <iostream>

namespace Gameplay
{
	class Window;
	class WindowEvaluator;

	namespace Behaviors
	{
		class Kick: public Behavior
		{
			public:
				Kick(GameplayModule *gameplay);
				~Kick();

				virtual void assign(std::set<Robot *> &available);
				virtual bool run();

				bool isIntercept()
				{
					return _state == Intercept;
				}

				Robot *targetRobot;
				bool automatic;

				enum State
				{
					Intercept,
					Aim,
					Shoot,
					Done
				};
				State getState() {return _state;}

				/** Restarts the kick play - keep going after ball */
				void restart() {_state = Intercept;}

			protected:
				virtual float score(Robot* robot);

				WindowEvaluator *_win;
				Gameplay::Behaviors::Intercept* _intercept;

				State _state;
				float _lastMargin;
				Geometry2d::Segment _target;

				//we lock in a pivot point
				Geometry2d::Point _pivot;

				// The robot's position when it entered the Shoot state
				Geometry2d::Point _shootStart;

				Geometry2d::Point _shootMove;
				Geometry2d::Point _shootBallStart;
		};
	}
}
