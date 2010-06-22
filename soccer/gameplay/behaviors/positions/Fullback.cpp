#include "Fullback.hpp"

#include <Constants.hpp>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include "../../Window.hpp"

#include <Geometry2d/util.h>

using namespace std;

Gameplay::Behaviors::Fullback::Fullback(GameplayModule *gameplay, Side side):
	Behavior(gameplay, 1),
	_side(side)
{
}

bool Gameplay::Behaviors::Fullback::assign(set<Robot *> &available)
{
	_robots.clear();
	if (!takeBest(available))
	{
	    return false;
	}
	
	//Initial state
	_state = Init;

	//initialize windowevaluator
	_winEval = boost::make_shared<Gameplay::WindowEvaluator>(Behavior::gameplay()->state());
	_winEval->debug = false;

	return _robots.size() >= _minRobots;
}

bool Gameplay::Behaviors::Fullback::run()
{
	if (!assigned() || !allVisible() || !_winEval)
	{
		return false;
	}
	
	// we multiply by 0.3 here to look 0.3s into the future when considering the ball position
	// also, this will be used for where the robots will face
	Geometry2d::Point ballFuture = ball().pos + ball().vel*0.3;

	//goal line, for intersection detection
	Geometry2d::Segment goalLine(Geometry2d::Point(-Constants::Field::GoalWidth / 2.0f, 0),
								  Geometry2d::Point(Constants::Field::GoalWidth / 2.0f, 0));

	// Update the target window
	_winEval->exclude.clear();
	_winEval->exclude.push_back(Behavior::robot()->pos());
	
	//exclude robots that arn't the fullback
	//_winEval->run(ball().pos, goalLine);
	
	BOOST_FOREACH(Fullback *f, otherFullbacks)
	{
		if (f->robot())
		{
			_winEval->exclude.push_back(f->robot()->pos());
		}
	}
	
	_winEval->run(ballFuture, goalLine);
	
	Window* best = 0;

	Behavior* goalie = _gameplay->goalie();
	
	bool needTask = false;
	
	//pick biggest window on appropriate side
	if (goalie && goalie->robot())
	{
		BOOST_FOREACH(Window* window, _winEval->windows)
		{
			if (_side == Left)
			{
				if (!best || window->segment.center().x < goalie->robot()->pos().x)
				{
					best = window;
				}
			}
			else if (_side == Right)
			{
				if (!best || window->segment.center().x > goalie->robot()->pos().x)
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
			Geometry2d::Segment seg(window->segment.center(), ball().pos);
			float newDist = seg.distTo(Behavior::robot()->pos());
			
			if (!best || newDist < bestDist)
			{
				best = window;
				bestDist = newDist;
			}				
		}
	}
	
	if (best /*&& ((_side==Left&&ball().pos.x>=0) || (_side==Right&&ball().pos.x<0))*/)
	{
		Geometry2d::Segment shootLine(ball().pos, ball().pos + ball().vel.normalized() * 7.0);
		
		Geometry2d::Segment& winSeg = best->segment;
		
		if (ball().vel.magsq() > 0.03 && winSeg.intersects(shootLine))
		{
			robot()->move(shootLine.nearestPoint(Behavior::robot()->pos()));
			robot()->faceNone();
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
					//Behavior::robot()->face(ballFuture);
					// Using regular pos rather than future because velocity approx on ball is not exact
					// enough and this leads to the robots turning backwards, towards the goal, when the
					// ball is shot at the goal.
					Behavior::robot()->face(ball().pos);
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
	
	// if needTask, face the ball
	if(needTask)
	{
		robot()->face(ball().pos, true);
	}

	// Turn dribbler on when ball is near
	if(ball().pos.y < Constants::Field::Length / 2)
	{
		robot()->dribble(255);
	}

	// If ball sensor is tripped and we are not facing towards the goal, fire.
	Geometry2d::Point backVec(1,0);
	Geometry2d::Point backPos(-Constants::Field::Width/2,0);
	Geometry2d::Point shotVec(ball().pos - robot()->pos());
	Geometry2d::Point shotPos(robot()->pos());
	Geometry2d::Point backVecRot(backVec.perpCCW());
	bool facingBackLine = (backVecRot.dot(shotVec) < 0);
	if(!facingBackLine)
	{
		robot()->kick(255);
	}

	/*
	if(needTask){
		//goal line, for intersection detection
		Geometry2d::Segment goalLine(Geometry2d::Point(-Constants::Field::GoalWidth / 2.0f, 0),
									  Geometry2d::Point(Constants::Field::GoalWidth / 2.0f, 0));
		//goal arc
		const float radius = .7;
		Geometry2d::Circle arc(Geometry2d::Point(), radius);

		//opponent with ball
		Robot* oppWithBall = 0;
		float minDist = 999;
		BOOST_FOREACH(Robot *r, _gameplay->opp)
		{
			float dist = r->pos().distTo(ball().pos);
			if(dist < minDist)
			{
				minDist = dist;
				oppWithBall = r;
			}
		}
		if(oppWithBall == 0)
			return false; // need opp to block


		// block opponents that are pointed towards our goal and are on our side (left or right) and do not have the ball
		BOOST_FOREACH(Robot *r, _gameplay->opp)
		{
			if(r == oppWithBall)
				continue;

			bool sameSide = ((_side==Left&&r->pos().x<=0) || (_side==Right&&r->pos().x>=0));
			bool nonDefender = (r->pos().y < 3*Constants::Field::Length/4);
			bool facingGoal;
			Geometry2d::Point facing(cos(DegreesToRadians * r->angle()),sin(DegreesToRadians * r->angle())); // make sure not degrees!
			facing *= 20;
			Geometry2d::Segment los(facing, r->pos());
			Geometry2d::Point intr;
			facingGoal = los.intersects(goalLine,&intr);

			if(sameSide && nonDefender && facingGoal)
			{
				Geometry2d::Point dest[2];
				Geometry2d::Line losLine(facing,r->pos());
				bool ballTravelIntersects = losLine.intersects(arc, &dest[0], &dest[1]);
				if(!ballTravelIntersects)
					continue;
				Geometry2d::Point blockPoint = (dest[0].y > 0 ? dest[0] : dest[1]);
				Behavior::robot()->move(blockPoint);
				Behavior::robot()->face(r->pos());
				return true;
			}
		}
	}*/
		/*
	if(needTask) //TODO: look at this in detail. Hacked together so that robots don't sit around
	{
		//if no side parameter...stay in the middle
		float bestDist = 0;
		BOOST_FOREACH(Window* window, _winEval->windows)
		{
			Geometry2d::Segment seg(window->segment.center(), ball().pos);
			float newDist = seg.distTo(Behavior::robot()->pos());

			if (!best || newDist < bestDist)
			{
				best = window;
				bestDist = newDist;
			}
		}
		Geometry2d::Segment shootLine(ball().pos, ball().pos + ball().vel.normalized() * 7.0);

		Geometry2d::Segment& winSeg = best->segment;

		robot()->move(shootLine.nearestPoint(Behavior::robot()->pos()));
		robot()->faceNone();
	}*/
	return false;
}

float Gameplay::Behaviors::Fullback::score(Robot* robot)
{
	//robot closest to the back line wins
	return robot->pos().y;
}
