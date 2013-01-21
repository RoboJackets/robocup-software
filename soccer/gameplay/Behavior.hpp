#pragma once

#include "GameplayModule.hpp"
#include <framework/SystemState.hpp>
#include <Configuration.hpp>
#include <Robot.hpp>

namespace Gameplay
{
	class Behavior
	{
		public:
			Behavior(GameplayModule *gameplay);
			virtual ~Behavior();

			GameplayModule *gameplay() const
			{
				return _gameplay;
			}
			
			SystemState *state() const
			{
				return _gameplay->state();
			}

			// Called each frame when this behavior is current.
			// The default implementation does nothing.
			//
			// Returns true if this behavior needs to continue or false if it is done and may be replaced by another behavior.
			// The behavior may continue to be used after run() returns false.
			virtual bool run() = 0;

		protected:
			GameplayModule *_gameplay;

			// Convenience functions
			const Ball &ball() const
			{
				return state()->ball;
			}

			const GameState &gameState() const
			{
				return state()->gameState;
			}

			OurRobot *self(int i) const
			{
				return state()->self[i];
			}

			const OpponentRobot *opp(int i) const
			{
				return state()->opp[i];
			}
	};
	
	class SingleRobotBehavior: public Behavior
	{
		public:
			SingleRobotBehavior(GameplayModule *gameplay)
				: Behavior(gameplay),
				  robot(0)
			{
			}
			
			OurRobot *robot;
	};

	class TwoRobotBehavior: public Behavior
	{
		public:
			TwoRobotBehavior(GameplayModule *gameplay)
				: Behavior(gameplay),
				  robot1(0),
				  robot2(0)
			{
			}

			OurRobot *robot1;
			OurRobot *robot2;
	};

	class ReceivingBehavior;

	class PassingBehavior : public SingleRobotBehavior
	{
	public:
		PassingBehavior(GameplayModule *gameplay)
			: SingleRobotBehavior(gameplay),
			  receiveTarget(0,0),
			  receiver(0)
		{
		}

		virtual bool done() = 0;
		virtual bool kicking() = 0;
		virtual bool setup() = 0;
		virtual void restart() = 0;

		Geometry2d::Point receiveTarget;

		ReceivingBehavior *receiver;
	};

	class ReceivingBehavior : public SingleRobotBehavior
	{
	public:
		ReceivingBehavior(GameplayModule *gameplay)
			: SingleRobotBehavior(gameplay),
			  receiveTarget(0,0),
			  passer(0)
		{
		}

		virtual bool done() = 0;
		virtual bool success() = 0;
		virtual bool ready() = 0;
		virtual void restart() = 0;

		Geometry2d::Point receiveTarget;

		PassingBehavior *passer;
	};
}
