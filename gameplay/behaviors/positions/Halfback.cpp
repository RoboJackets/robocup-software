#include "Halfback.hpp"
#include "Fullback.hpp"
#include "Striker.hpp"

#include <Constants.hpp>
#include <LogFrame.hpp>
#include <vector>
#include "../../Predicates.hpp"

#include <iostream>
using namespace std;
using namespace Geometry2d;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Halfback> behavior("halfback");

Gameplay::Behaviors::Halfback::Halfback(GameplayModule *gameplay, Role *role):
	Behavior(gameplay, role),
	RobotNode(gameplay),
	side_param(this, "side")
{
	side_param.legal.push_back("left");
	side_param.legal.push_back("center");
	side_param.legal.push_back("right");
}

void Gameplay::Behaviors::Halfback::start()
{
	_state = Init;

	//Initialize skills
	_mark = new Gameplay::Behaviors::Mark(Behavior::gameplay());
	_intercept = new Gameplay::Behaviors::Intercept(Behavior::gameplay());
	_kick = new Gameplay::Behaviors::Kick(Behavior::gameplay());
	_receive = new Gameplay::Behaviors::ReceivePass(Behavior::gameplay());
	_mark->robot(Behavior::robot());
	_intercept->robot(Behavior::robot());
	_kick->robot(Behavior::robot());
	_receive->robot(Behavior::robot());

}

void Gameplay::Behaviors::Halfback::run()
{

	//Copy in important state data
	const Packet::LogFrame::Ball& b = ball();
	Geometry2d::Point ball_pos = b.pos;

	//Determine cases
	bool offense = Gameplay::Predicates::offense.value;
	bool free_ball = Gameplay::Predicates::free_ball.value;


	//check for incoming pass
	if (_receiveFrom != 0)
	{
		//drop everything else and receive pass
		_state = Receiving;
		_receive->start();
	}

	//State machine
	if (_state == Init)
	{
		if (!offense)
		{ //just attempt to attack the ball
			_intercept->start();
			_state = Intercept;
		}
		else
		{
			//find a pass if has the ball
			if (Behavior::robot()->state()->haveBall)
			{
				GraphNode * target  = nextNode();
				if (target != 0 && typeid(*target) == typeid(RobotNode))
				{
					//pass the ball
					_state = Passing;
					Robot * r = dynamic_cast<RobotNode * >(target)->robot;
					_kick->target_param.set(r);

					//direct the target robot to be ready for a pass
					Behavior *b = r->behavior();
					if (b)
					{
						if (typeid(*b) == typeid(Fullback))
						{
							Fullback * rb = dynamic_cast<Fullback*>(b);
							rb->_receiveFrom = Behavior::robot();
						}
						else if (typeid(*b) == typeid(Halfback))
						{
							Halfback * rb = dynamic_cast<Halfback*>(b);
							rb->_receiveFrom = Behavior::robot();
						}
						else if (typeid(*b) == typeid(Striker))
						{
							Striker * rb = dynamic_cast<Striker*>(b);
							rb->_receiveFrom = Behavior::robot();
						}
					}
				}
				else if (target != 0 && typeid(*target) == typeid(GoalNode))
				{
					//shoot on goal
					_state = Shooting;
					_kick->target_param.clear();
				}
			}
			// if robot does not have ball, find a good place to go via RRT
			// want to connect the ball node to someone with a shot on goal
			else
			{
				_state = Intercept;
				_intercept->start();
			}
		}

	}

	//Chase the ball madly
	else if (_state == Intercept)
	{
		_intercept->run();
		if (_intercept->done())
		{
			_state = Init;
		}
	}
	//Stay in zone and look for forward pass angles
	else if (_state == Support)
	{
		//find a good place to go to
		//score for the node is the combined sizes of the window segments

		//		rrt - call init(start, obstacles(gameplayrobot))
		//		call extend repeatedly
		//		search over points for good
	}
	//Incoming pass - use passing skill
	else if (_state == Receiving)
	{
		_receive->run();
		if (_receive->done())
		{
			_state = Init;
			_receiveFrom = 0; //reset receive porting
		}

	}
	//Passing to another robot - using Kick skill
	else if (_state == Passing)
	{
		_kick->run();
		if (_kick->done())
		{
			_state = Init;
		}
	}
	//Shooting on goal - using Kick skill
	else if (_state == Shooting)
	{
		//run shooting command
		_kick->run();
		//check for end
		if (_kick->done())
		{
			_state = Init;
		}

	}

}


float Gameplay::Behaviors::Halfback::score(Robot* robot)
{

	float dist = robot->pos().distTo(Point());
	if (side_param.valid() && side_param.value() != "center")
	{
		Point left(Constants::Field::Width/2, Constants::Field::Length/2);
		Point right(-Constants::Field::Width/2, Constants::Field::Length/2);
		if (side_param.value() == "left")
		{
			dist += 1.3*(robot->pos().distTo(left));
		}
		else
		{
			dist += 1.3*(robot->pos().distTo(right));
		}
	}
	return dist;

}

