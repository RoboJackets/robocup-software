#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Intercept: public Behavior
		{
			public:
				Intercept(GameplayModule *gameplay);

				virtual void run();
				virtual void start();
				virtual bool done();

				/** Closest robot to the ball gets
				 * the intercept */
				virtual float score(Robot * robot);

				Geometry2d::Point target;
				
			protected:
				typedef enum
				{
					ApproachFar, 
					ApproachBall, 
					Done
				} State;

				State _state;
				
		};
	}
}
