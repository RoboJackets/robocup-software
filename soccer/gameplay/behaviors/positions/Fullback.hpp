#pragma once

#include "../../Behavior.hpp"
#include "../../Window.hpp"

#include <boost/shared_ptr.hpp>

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

				Fullback(GameplayModule *gameplay, Side side = Center);

				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
				
				void side(Side s) { _side = s; }
				Side side() const { return _side; }

				std::set<Fullback *> otherFullbacks;
				
			protected:
				Side _side;
				
				//Window evaluator for checking passes/shots
				boost::shared_ptr<Gameplay::WindowEvaluator> _winEval;

				//state information
				State _state;

				virtual float score(Robot *r);
		};
	}
}
