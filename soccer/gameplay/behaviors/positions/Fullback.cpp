#include "Fullback.hpp"
#include "Halfback.hpp"
#include "Striker.hpp"
#include <Constants.hpp>
#include <LogFrame.hpp>
#include <vector>

#include <boost/foreach.hpp>

#include "../../Predicates.hpp"
#include "../../Window.hpp"

#include <iostream>
#include <cstdlib>

using namespace std;
using namespace Geometry2d;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Fullback> behavior("fullback");

Gameplay::Behaviors::Fullback::Fullback(GameplayModule *gameplay, Role *role):
	Behavior(gameplay, role),
	RobotNode(gameplay),
	side_param(this, "side")
{
	side_param.legal.push_back("left");
	side_param.legal.push_back("center");
	side_param.legal.push_back("right");

	//set pass data to no passes
	_receiveFrom = 0;
	_passTarget = 0;
}

void Gameplay::Behaviors::Fullback::start()
{
	//Initial state
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

	//initialize windowevaluator
	_winEval = new Gameplay::WindowEvaluator(Behavior::gameplay()->state());
	_winEval->debug = false;

#if 0
	//Zone is defined by one L/R half of the field, and a third the
	// length of the field.  Tune this to adjust response
	//New change: zones overlap in the middle
	if (!Gameplay::Predicates::offense.value) //Defense
	{
		if (side_param.value() == "right")
		{
			cout << "creating right zone" << endl;
			_zone = Geometry2d::Rect(Geometry2d::Point(-0.3, 0),
					Geometry2d::Point(Constants::Field::Width /2,
							Constants::Field::Length /3));
			_homePos = Geometry2d::Point(1, 0.6);
		}
		else if (side_param.value() == "left")
		{
			cout << "creating left zone" << endl;
			_zone = Geometry2d::Rect(Geometry2d::Point(0.3,0),
					Geometry2d::Point(-1*Constants::Field::Width /2,
							Constants::Field::Length /3));
			_homePos = Geometry2d::Point(-1, 0.6);
		}
		else
		{ //for center, cover whole backfield
			cout << "creating backfield" << endl;
			_zone = Geometry2d::Rect(Geometry2d::Point(-1*Constants::Field::Width/2,
					0),
					Geometry2d::Point(Constants::Field::Width/2,
							Constants::Field::Length/3));
			_homePos = Geometry2d::Point(0, Constants::Field::Length/6);
		}
	}
	else
	{	// Offense - half field zone, home position at 1/3 field edge
		if (side_param.value() == "right")
		{
			cout << "creating right zone" << endl;
			_zone = Geometry2d::Rect(Geometry2d::Point(),
					Geometry2d::Point(Constants::Field::Width /2,
							Constants::Field::Length /2));
			_homePos = Geometry2d::Point(1, Constants::Field::Length/3);
		}
		else if (side_param.value() == "left")
		{
			cout << "creating left zone" << endl;
			_zone = Geometry2d::Rect(Geometry2d::Point(),
					                 Geometry2d::Point(-1*Constants::Field::Width /2,
							                           Constants::Field::Length /3));
			_homePos = Geometry2d::Point(-1, Constants::Field::Length/3);
		}
		else
		{ //for center, cover whole backfield
			cout << "creating backfield" << endl;
			_zone = Geometry2d::Rect(Geometry2d::Point(-1*Constants::Field::Width/2,
													   0),
									 Geometry2d::Point(Constants::Field::Width/2,
							                           Constants::Field::Length/2));
			_homePos = Geometry2d::Point(0, Constants::Field::Length/3);
		}
	}
#endif
}

void Gameplay::Behaviors::Fullback::run()
{
	//Copy in important state data
	const Packet::LogFrame::Ball& b = ball();
	
	Point ballFuture = b.pos + b.vel;

	//goal line, for intersection detection
	Segment goalLine(Point(-Constants::Field::GoalWidth / 2.0f, 0),
			Point(Constants::Field::GoalWidth / 2.0f, 0));

	// Update the target window
	_winEval->exclude.clear();
	_winEval->exclude.push_back(Behavior::robot()->pos());
	
	//exclude robots that arn't the fullback
	//_winEval->run(b.pos, goalLine);
	
	std::list<Robot *> robots;
	Behavior::gameplay()->find_not_of_type<Gameplay::Behaviors::Fullback*>(robots);
	
	BOOST_FOREACH(Robot* r, robots)
	{
		_winEval->exclude.push_back(r->pos());
	}
	
	_winEval->run(ballFuture, goalLine);
	
	Window* best = 0;

	Behavior* gb = Behavior::gameplay()->playbook.goalie();
	
	bool needTask = false;
	
	//pick biggest window on appropriate side
	if (side_param.valid() && gb && gb->robot())
	{
		BOOST_FOREACH(Window* window, _winEval->windows)
		{
			if (side_param.value() == "left")
			{
				if (!best || window->segment.center().x < gb->robot()->pos().x)
				{
					best = window;
				}
			}
			else if (side_param.value() == "right")
			{
				if (!best || window->segment.center().x > gb->robot()->pos().x)
				{
					best = window;
				}
			}
		}
	}
	else
	{
		//if no side parameter...stay in the middle
		float bestDist = 0;
		BOOST_FOREACH(Window* window, _winEval->windows)
		{
			Geometry2d::Segment seg(window->segment.center(), b.pos);
			float newDist = seg.distTo(Behavior::robot()->pos());
			
			if (!best || newDist < bestDist)
			{
				best = window;
				bestDist = newDist;
			}				
		}
	}
	
	if (best)
	{
		Geometry2d::Segment shootLine(b.pos, b.pos + b.vel.normalized() * 7.0);
		
		Geometry2d::Segment& winSeg = best->segment;
		
		if (b.vel.magsq() > 0.03 && winSeg.intersects(shootLine))
		{
			Behavior::robot()->move(shootLine.nearestPoint(Behavior::robot()->pos()));
			Behavior::robot()->state()->cmd.face = Packet::LogFrame::MotionCmd::None;
		}
		else
		{
			const float winSize = winSeg.length();
				
			if (winSize < Constants::Ball::Radius)
			{
				needTask = true;
			}
			else
			{
				const float radius = .7;
				
				Geometry2d::Circle arc(Geometry2d::Point(), radius);
				
				Geometry2d::Line shot(winSeg.center(), ballFuture);
				Geometry2d::Point dest[2];
				
				bool intersected = shot.intersects(arc, &dest[0], &dest[1]);
				
				if (intersected)
				{
					if (dest[0].y > 0)
					{
						Behavior::robot()->move(dest[0]);
					}
					else
					{
						Behavior::robot()->move(dest[1]);
					}
					Behavior::robot()->face(ballFuture);
				}
				else
				{
					needTask = true;
				}
			}
		}
	}
	else
	{
		needTask = true;
	}

#if 0
	if (needTask)
	{
		std::vector<Gameplay::Robot*> possibles;
		
		//Determine opponents in zone
		Robot** opps = Behavior::gameplay()->opp;
		
		for (int i = 0; i<Constants::Robots_Per_Team; ++i)
		{
			Gameplay::Robot* opp = opps[i];
			
			bool ok = true;
			BOOST_FOREACH(Packet::LogFrame::Robot& self, Behavior::gameplay()->state()->self)
			{
				//if anyone is near the opponent
				if (opp->state()->valid && self.valid && opp->pos().nearPoint(self.pos, .5))
				{
					ok = false;
					break;
				}
			}
			
			if (ok)
			{
				possibles.push_back(opp);
			}
		}
		
		Gameplay::Robot* mark = 0;
		
		//find the closest robot to mark
		BOOST_FOREACH(Gameplay::Robot* r, possibles)
		{
			if (!mark || r->pos().y < mark->pos().y)
			{
				mark = r;
			}
		}
		
		if (mark && mark->pos().y < Constants::Field::Length/2.0f)
		{
			//run mark behavior
			_mark->target_param.set(mark);
			_mark->run();
		}
	}
#endif
	
#if 0
	//Determine opponents in zone
	Robot **opp_full = Behavior::gameplay()->opp;
	std::vector<Robot*> opp_in_zone;
	for (int i = 0; i<Constants::Robots_Per_Team; ++i)
	{
		if (opp_full[i]->state()->valid && _zone.contains(opp_full[i]->pos()))
			opp_in_zone.push_back(opp_full[i]);
	}

	//Determine cases
	int shooters_in_zone = opp_in_zone.size();
	bool ball_in_zone = _zone.contains(ball_pos);
	bool offense = Gameplay::Predicates::offense.value;
	bool free_ball = Gameplay::Predicates::free_ball.value;

	//check for incoming pass
	if (_receiveFrom != 0)
	{
		//drop everything else and receive pass
		_state = Receiving;
	}

	//state machine
	if (_state== Init)
	{
		//determine state of system and choose tactics
		if (!offense)
		{ //defensive plays
			if (free_ball && ball_in_zone && shooters_in_zone == 0)
			{
				//intercept the free ball
				_intercept->start();
				_state = Intercept;
			}
			if (!free_ball && shooters_in_zone == 1 && ball_in_zone)
			{ //for single shooter, block the goal directly with up close marking
				_mark->target_param.set(opp_in_zone[0]);
				_mark->coverGoal_param.set(true);
				_state = Marking;
			}
			if (shooters_in_zone > 0 && !ball_in_zone)
			{
				//mark this robot
				_state = MultiMark;
			}
			else if (shooters_in_zone > 1 && ball_in_zone && !free_ball)
			{
				//make attempt to intercept
				//FIXME: switch this to steal
				_intercept->start();
				_state = Intercept;
			}
			else if (shooters_in_zone == 0 && ball_in_zone)
			{
				cout << "Intercepting Ball" << endl;
				//get the free ball
				_intercept->start();

				//switch state
				_state = Intercept;
			}
			//default behavior: goto goal arc covering
			else
			{
				Behavior::robot()->move(_homePos);
				Behavior::robot()->face(ball_pos);
			}
		}
		else
		{ //offensive plays
			//if have ball, make a pass
			if (Behavior::robot()->state()->haveBall)
			{
				//find a pass
				GraphNode * target  = nextNode();
				if (target != 0 && typeid(*target) == typeid(RobotNode))
				{
					//pass the ball
					_state = Passing;
					Robot * r = dynamic_cast<RobotNode * >(target)->robot;
					_kick->target_param.set(r);

					//direct the target robot to be ready for a pass
					if (typeid(*(r->behavior())) == typeid(Fullback))
					{
						Fullback * rb = dynamic_cast<Fullback*>(r->behavior());
						rb->_receiveFrom = Behavior::robot();
					}
					else if (typeid(*(r->behavior())) == typeid(Halfback))
					{
						Halfback * rb = dynamic_cast<Halfback*>(r->behavior());
						rb->_receiveFrom = Behavior::robot();
					}
					else if (typeid(*(r->behavior())) == typeid(Striker))
					{
						Striker * rb = dynamic_cast<Striker*>(r->behavior());
						rb->_receiveFrom = Behavior::robot();
					}
				}
			}
			if (free_ball && _zone.contains(ball_pos))
			{
				//intercept the ball if it is free and in the zone
				_state = Intercept;
				_intercept->start();
			}
			else
			{
				//play supporting role
				_state = Support;
			}

		}
	}
	//guard a single robot, while staying in zone
	if (_state == Marking)
	{
		//run mark behavior
		_mark->run();

		//check if necessary to switch behaviors
		Robot * target = _mark->target_param.robot();
		// if opp has left the zone, retreat
		// if robot has left the zone, retreat
		if (!_zone.contains(target->pos()) ||
				!_zone.contains(Behavior::robot()->pos()) ||
				shooters_in_zone > 1)
		{
			_state = Init;
		}
	}
	//Switch between closest target to mark
	if (_state == MultiMark)
	{
		if (shooters_in_zone == 0 || ball_in_zone)
		{
			_state = Init;
		}
		else
		{
			//mark the closest shooter to the goal
			Robot * closest = opp_in_zone[0];

			for (unsigned int i = 1; i < opp_in_zone.size(); ++i)
			{
				//Project the opp position
				Robot * r = opp_in_zone[i];
				Point opp_proj = r->pos() + r->vel() * 0.5;
				Point close_proj = closest->pos() + closest->vel() * 0.5;

				//find closest point to goal
				Segment goal_line(Point(Constants::Field::GoalWidth/2, 0),Point(-Constants::Field::GoalWidth/2, 0));
				Point goal_point = goal_line.nearestPoint(opp_proj);
				Point goal_pt_closest = goal_line.nearestPoint(close_proj);

				if (opp_proj.distTo(goal_point) < close_proj.distTo(goal_pt_closest))
				{
					closest = r;
				}
			}
			_mark->target_param.set(closest);
			_mark->coverGoal_param.set(false);
			_mark->run();
		}
	}
	//Defensive Intercept, stays in zone
	if (_state == Intercept)
	{
		//execute intercept behavior
		_intercept->run();

		//check if necessary to switch
		if (_intercept->done())
		{
			//if the robot catches the ball, defense over
			_state = Init;
		}
		else if (!_zone.contains(Behavior::robot()->pos()) ||
				!_zone.contains(ball_pos) ||
				shooters_in_zone > 0)
		{
			_state = Init;
		}
	}
	//Hold back in zone to allow for back passes
	if (_state == Support)
	{




		//TODO: Implement RRT for finding a support location
		//calculating best position for a back pass to the ball
		//
//		Point total_best = robot()->pos();
//		float step_size = 0.5;
//		unsigned int max_iter = 20;
//		for (unsigned int i = 0; i < max_iter; ++i)
//		{
//			//generate a new (viable) point
//			float angle = (float) 2*M_PI*drand48();
//			Point pt = total_best.direction(angle) * step_size;
//			//score new point
//
//			//if better than the previous best, replace
//		}
		//Loop over target number of iterations
		//pick a random point at a given range
		//develop highest scoring node



	}
	//Receiving a pass - using Receive skill
	if (_state == Receiving)
	{
		//TODO: Implement this
		//NOTE: This is only called when the passer forces it to call
		//Set receive skill to receive from passer
		//run until completion
	}
	//Sending a pass  - using Pass skill
	if (_state == Passing)
	{
		//TODO: Implement this
		//find an open target
			//otherwise, do nothing
		//set target's receiveFrom to this robot
		//turn on passing module
		//if passing module done, clean up
	}
#endif
}


float Gameplay::Behaviors::Fullback::score(Robot* robot)
{
	//robot closest to the back line wins
	return robot->pos().y;

	//score is the sum of the distance to the goal and the zone's corner
	float goal_dist = robot->state()->pos.distTo(Geometry2d::Point());
	if (side_param.value() != "center")
	{
		Geometry2d::Point corner(Constants::Field::Width / 2, 0);
		if (side_param.value() == "left")
		{
			corner = corner * -1;
		}
		float corner_dist = robot->state()->pos.distTo(corner);
		return goal_dist+corner_dist;
	}
	else
	{
		return goal_dist;
	}
}

