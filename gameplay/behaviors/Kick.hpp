#pragma once

#include "../Behavior.hpp"

#include "../parameters/Robot_Parameter.hpp"
#include "../parameters/String_Parameter.hpp"

#include "Intercept.hpp"

namespace Gameplay
{
	class Window;

	namespace Behaviors
	{
		class Kick: public Behavior
		{
			public:
				Kick(GameplayModule *gameplay, Role *role = 0);
				~Kick();

				virtual void start();
				virtual void run();
				virtual bool done();

				virtual float score(Robot* robot);
				bool isIntercept()
				{
					return _state == Intercept;
				}

				Robot_Parameter target_param;
				String_Parameter mode_param;

			protected:
				enum State
				{
					Intercept,
					Aim,
					Shoot,
					Done
				};

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
