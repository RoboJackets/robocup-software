#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/Window.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class Kick: public SingleRobotBehavior
		{
			public:
				Kick(GameplayModule *gameplay);
				
				virtual bool run();

				inline bool done() const { return _state == State_Done; }
				
				void restart();
				
				// Kicks at the opponent's goal
				void setTargetGoal();
				
				// Kicks at a robot (pass)
				void setTarget(Robot *r);
				
				void setTarget(const Geometry2d::Segment &seg);
			
			private:
				enum State
				{
					State_Approach1,
					State_Approach2,
					State_Aim,
					State_Kick,
					State_Done
				};
				
				State _state;
				float _lastError;
				
				Geometry2d::Segment _target;
				
				// Used to display expected kick trajectory after finishing
				Geometry2d::Segment _kickSegment;

                                bool hasShot;
		};
	}
}
