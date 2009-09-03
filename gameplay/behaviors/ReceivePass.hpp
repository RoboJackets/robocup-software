#pragma once

#include "../Behavior.hpp"
#include "../parameters/Float_Parameter.hpp"
#include "../parameters/Robot_Parameter.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Kick;

		class ReceivePass: public Behavior
		{
			public:
				ReceivePass(GameplayModule *gameplay, Role *role = 0);
				~ReceivePass();

				virtual void start();
				virtual void run();
				virtual bool done();

			protected:
				enum State
				{
					Wait,
					Receive,
					Kick
				};

				State _state;

				Gameplay::Behaviors::Kick *_kick;

				// Number of consecutive frames in which the ball was moving towards the robot in the Wait state.
				int _waitCount;

				Geometry2d::Point _lastPos;

				Float_Parameter range_param;
				Robot_Parameter target_param;
		};
	}
}
