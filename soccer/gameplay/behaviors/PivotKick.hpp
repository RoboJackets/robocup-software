#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/Capture.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class PivotKick: public SingleRobotBehavior
		{
			public:
				PivotKick(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				bool aiming() const {
					return _state == State_Aim;
				}

				bool capture() const {
					return _state == State_Capture;
				}

				void restart();
				
				Geometry2d::Segment target;
				
			private:

				enum
				{
					State_Capture,
					State_Aim,
					State_Done
				} _state;
				
				Capture _capture;

				bool _ccw;
				float _lastError;
				float _lastDelta;
				uint64_t _lastBallTime;
				float _accuracy;
				bool _kicked;
			};
	}
}
