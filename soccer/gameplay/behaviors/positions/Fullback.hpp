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
				static void createConfiguration(Configuration *cfg);

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

				virtual bool run();

				/** set the robot to block the ball */
				void blockBall(bool b){ _blockBall = b; }

				/** set the robot to block an opp robot */
				void blockRobot(OpponentRobot * robot) { _blockRobot = robot; _blockBall = false; }
				OpponentRobot * blockRobot() const { return _blockRobot;}

				void side(Side s) { _side = s; }
				Side side() const { return _side; }

				std::set<Fullback *> otherFullbacks;
				
				OurRobot *robot;
				OpponentRobot *_blockRobot;

			protected:
				//Window evaluator for checking passes/shots
				Gameplay::WindowEvaluator _winEval;

				Side _side;
				State _state;
				/** defaults to true */
				bool _blockBall;
		};
	}
}
