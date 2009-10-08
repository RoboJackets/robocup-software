#pragma once

#include "../../Behavior.hpp"
#include "../../GraphNode.hpp"
#include "../Mark.hpp"
#include "../Intercept.hpp"
#include "../ReceivePass.hpp"
#include "../Kick.hpp"
#include "../../parameters/String_Parameter.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Mark;
		class Intercept;
		class ReceivePass;
		class Kick;

		class Fullback: public Behavior, public RobotNode
		{
			public:

				typedef enum
				{
					//defense states
					Init,
					Marking,
					MultiMark,
					Intercept,
					//offensive states
					Support,
					Receiving,
					Passing
				} State;

				Fullback(GameplayModule *gameplay, Role *role);

				virtual void run();
				virtual void start();
				virtual float score(Robot* robot);

				//Receive From (robot passing to this robot)
				Robot * _receiveFrom;

				//Pass target (robot this robot is passing to)
				Robot * _passTarget;

			protected:
				//Parameter to determine if the robot is in the right zone
				String_Parameter side_param;

				//Zone definition
				Geometry2d::Rect _zone; //Default zone in which the robot operates
				Geometry2d::Point _homePos; //Home position for the robot
				
				//Mark Behavior
				Gameplay::Behaviors::Mark * _mark;
				//Intercept Behavior
				Gameplay::Behaviors::Intercept * _intercept;
				//Receive Pass behavior
				Gameplay::Behaviors::ReceivePass * _receive;
				//Kick/pass behavior
				Gameplay::Behaviors::Kick * _kick;

				//Window evaluator for checking passes/shots
				Gameplay::WindowEvaluator * _winEval;

				//state information
				State _state;

		};
	}
}
