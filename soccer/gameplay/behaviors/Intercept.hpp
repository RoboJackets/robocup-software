#pragma once

#include "../Behavior.hpp"
#include "../parameters/Point_Parameter.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Intercept: public Behavior
		{
			public:
				
				Intercept(GameplayModule *gameplay, Role *role = 0);

				virtual void run();
				virtual void start();
				virtual bool done();

				/** Closest robot to the ball gets
				 * the intercept */
				virtual float score(Robot * robot);

				Point_Parameter target_param;
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
