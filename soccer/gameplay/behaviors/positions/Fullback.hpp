#pragma once

#include "../../Behavior.hpp"
#include "../../Window.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Fullback: public Behavior
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

				typedef enum
				{
					Left,
					Center,
					Right
				} Side;

				Fullback(GameplayModule *gameplay, Side side);

				virtual void assign(std::set<Robot *> &available);
				virtual bool run();
				
				std::set<Fullback *> otherFullbacks;
				
			protected:
				Side _side;
				
				//Window evaluator for checking passes/shots
				Gameplay::WindowEvaluator * _winEval;

				//state information
				State _state;

				virtual float score(Robot *r);
		};
	}
}
