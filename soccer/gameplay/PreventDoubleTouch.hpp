#pragma once

#include <gameplay/behaviors/Idle.hpp>
#include <QTime>

namespace Gameplay
{
	class PreventDoubleTouch
	{
		public:
			PreventDoubleTouch(GameplayModule *gameplay, SingleRobotBehavior *kicker = 0);
			
			void run();
			
			bool keepRunning() const
			{
				return _keepRunning;
			}
			
			bool kicked() const
			{
				return _kicked;
			}

			void resetBehavior(SingleRobotBehavior *kicker)
			{
				_kicker = kicker;
			}

			Behaviors::Idle backoff;
			
		protected:
			SystemState *state() const
			{
				return _gameplay->state();
			}
			
			GameplayModule *_gameplay;
			SingleRobotBehavior *_kicker;
			bool _keepRunning;
			bool _kicked;
			bool _wasReady;
			QTime _ruleTime;
	};
}
