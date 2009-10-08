#pragma once

#include "../Behavior.hpp"
#include "../parameters/Robot_Parameter.hpp"
#include "../parameters/Bool_Parameter.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Mark: public Behavior
		{
			public:

				Mark(GameplayModule *gameplay, Role *role = 0);

				virtual void run();

				//target robot
				Robot_Parameter target_param;

				//cover goal
				Bool_Parameter coverGoal_param;

		};
	}
}
