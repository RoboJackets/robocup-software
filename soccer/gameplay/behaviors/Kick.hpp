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
				
				void override(bool override) { _override = override; }
                                
                                // Kicks at the opponent's goal
				void setTargetGoal();
				
				// Kicks at a robot (pass)
				void setTarget(Robot *r);
				
				void setTarget(const Geometry2d::Segment &seg);
	                        
                                //Calculates where the robot should go in the case of a moving ball
                                Geometry2d::Point calculateInterceptPoint();

			private:
				enum State
				{
					State_Approach1,
					State_Face, //Aligns the Robot's Dribbler with the ball (Eventually should be part of Approach1)
                                        State_Approach2,
					State_Aim,
					State_Kick,
					State_Done
				};
				
				State _state;
				float _lastError;
			        int _faceTimeout;
                                int _aimTimeout;
                                bool _override;

				Geometry2d::Segment _target;
			
                                //The best segment of the _target that is able to be shot at
                                Geometry2d::Segment _shotSegment;

				// Used to display expected kick trajectory after finishing
				Geometry2d::Segment _kickSegment;

                                bool hasShot;
		};
	}
}
