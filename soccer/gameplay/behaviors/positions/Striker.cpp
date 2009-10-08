#include "Halfback.hpp"
#include "Fullback.hpp"
#include "Striker.hpp"

#include <Constants.hpp>
#include <LogFrame.hpp>
#include <vector>
#include <boost/foreach.hpp>

#include "../../../motion/planning/rrt.hpp"

#include "../../Predicates.hpp"
#include "../../Window.hpp"
#include "../../GameplayUtils.hpp"

#include <iostream>
using namespace std;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Striker> behavior("striker");

Gameplay::Behaviors::Striker::Striker(GameplayModule *gameplay, Role *role):
	Behavior(gameplay, role),
	RobotNode(gameplay),
	side_param(this, "side")
{
	side_param.legal.push_back("left");
	side_param.legal.push_back("center");
	side_param.legal.push_back("right");

	//check offense/defense
	bool offense = Gameplay::Predicates::offense.value;

	//set zone

	//set home point
}

void Gameplay::Behaviors::Striker::start()
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

void Gameplay::Behaviors::Striker::run()
{
	//Copy in important state data
	const Packet::LogFrame::Ball& b = ball();
	Geometry2d::Point ball_pos = b.pos;

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
						Fullback * rb = dynamic_cast<Fullback*>(r->behavior());
						rb->_receiveFrom = Behavior::robot();
					}
					else if (typeid(*b) == typeid(Halfback))
					{
						Halfback * rb = dynamic_cast<Halfback*>(r->behavior());
						rb->_receiveFrom = Behavior::robot();
					}
					else if (typeid(*b) == typeid(Striker))
					{
						Striker * rb = dynamic_cast<Striker*>(r->behavior());
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
		else if (Gameplay::Predicates::free_ball.value && ball_pos.y > Constants::Field::Length/2)
		{ // go after a free ball
			_state = Intercept;
			_intercept->start();
		}
		else //not a free ball, need to get into a position to shoot and receive a pass
		{
			_state = Support;
			_bestPos = Behavior::robot()->pos();
		}
	}

	//Intercept a free ball - use passing skill
	else if (_state == Intercept)
	{
		_intercept->run();
		if (Behavior::robot()->state()->haveBall)
		{
			_state = Init;
		}
	}
	//Stay in zone and look for forward pass angles
	else if (_state == Support)
	{
		//calculate RRT-based determination of best place to go
		// score with intent to receive a pass and score with it
		
		printf("support\n");
		Motion::FixedStepTree tree;
		tree.init(Behavior::robot()->pos(), &Behavior::robot()->state()->obstacles);
		for (int i = 0; i < 20; ++i)
		{
			tree.extend(Motion::RRT::randomPoint());
		}
		
		WindowEvaluator win(Behavior::_gameplay->state());
		win.exclude.push_back(Behavior::robot()->pos());
		win.debug = true;
		_bestPos = bestPassInTree(win, tree, Behavior::robot()->pos(), ball_pos);
		
		Behavior::robot()->move(_bestPos);
		Behavior::robot()->face(ball_pos);
		
		if (!Gameplay::Predicates::free_ball.value)
		{
			_state = Init;
		}
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

float Gameplay::Behaviors::Striker::score(Robot* robot)
{
	return robot->pos().y;
}
