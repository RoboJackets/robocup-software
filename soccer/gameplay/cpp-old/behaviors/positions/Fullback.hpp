#pragma once

#include "../../Behavior.hpp"
#include "../../evaluation/WindowEvaluator.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Defender: public Behavior
		{
			public:
				static void createConfiguration(Configuration *cfg);

				enum  Role
				{
					//defense states
					Marking = 1,
					AreaMarking = 2,
					MultiMark = 4,
					Intercept = 8,
					//offensive states
					Support = 16,
					Receiving = 32,
					Passing = 64
				};

				typedef enum
				{
					Left,
					Center,
					Right
				} Side;

				Defender(GameplayModule *gameplay, Side side = Center, int role = Marking);

				virtual bool run();

				/** set the robot to block the ball */
				void blockBall(){ _blockRobot = 0; }

				/** set the robot to block an opp robot */
				void blockRobot(OpponentRobot * robot) { _blockRobot = robot; }
				OpponentRobot * blockRobot() const { return _blockRobot;}

				void side(Side s) { _side = s; }
				Side side() const { return _side; }

				void role(int r) { _roles = r; }
				int role() const { return _roles; }

				std::set<Defender *> otherDefenders;
				
				OurRobot *robot;
				OpponentRobot *_blockRobot;

			protected:
				//Window evaluator for checking passes/shots
				Gameplay::WindowEvaluator _winEval;

				Side _side;
				int _roles;
				Role _state;

				static ConfigDouble *_defend_goal_radius;
				static ConfigDouble *_opponent_avoid_threshold;

				OpponentRobot* findRobotToBlock(const Geometry2d::Rect& area);
		};
	}
}
