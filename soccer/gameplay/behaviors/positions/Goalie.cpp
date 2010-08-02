// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

//FIXME - The rules allow for changing the goalie only in certain circumstances.  Make sure we do this right.

#include "Goalie.hpp"

#include <gameplay/Window.hpp>

#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

static const float MaxX = Constants::Field::GoalWidth / 2.0f;

Gameplay::Behaviors::Goalie::Goalie(GameplayModule *gameplay):
	Behavior(gameplay),
	_kick(gameplay)
{
	_win = 0;
	_state = Defend;
}

Gameplay::Behaviors::Goalie::~Goalie()
{
	delete _win;
}

bool Gameplay::Behaviors::Goalie::assign(set<Robot *> &available)
{
	_robots.clear();
	if (!takeBest(available))
	{
	    return false;
	}
	
	if (!_win)
	{
		_win = new WindowEvaluator(_gameplay->state());
		_win->debug = true;
	}
	
	_kick.assignOne(robot());
	
	_state = Defend;
	
	_index = 0;

	return true;
}

float Gameplay::Behaviors::Goalie::score(Robot *r)
{
	return r->pos().magsq();
}

bool Gameplay::Behaviors::Goalie::run()
{
	if (!assigned() || !allVisible())
	{
		return true;
	}

	if (!ball().valid)
	{
		robot()->move(Geometry2d::Point(0, Constants::Robot::Radius));
		return true;
	}
	
	if (_state == Clear)
	{
		//if the ball is in the defense area...clear it
		_kick.setTarget();
		_kick.automatic = true;
		
		robot()->dribble(50);
		
		bool done = !_kick.run();
		
		//done or give up
		//if ball in goal...don't go in goal
		if (done || ball().pos.mag() > .75 || ball().pos.y < 0)
		{
			_state = Defend;
		}
	}
	else if (_state == Defend)
	{
		robot()->face(ball().pos);

		Robot* closest = 0;
		float bestDist = 0;
		BOOST_FOREACH(Robot *r, gameplay()->opp)
		{
			float dist = r->pos().distTo(ball().pos);
			if (!closest || dist < bestDist)
			{
				closest = r;
				bestDist = dist;
			}
		}

		//const Point goalBack(0,-Constants::Field::GoalDepth);

		//goal line, for intersection detection
		Segment goalLine(Point(-MaxX, 0), Point(MaxX, 0));
		
		// Update the target window
		_win->exclude.clear();
		_win->exclude.push_back(robot()->pos());
		_win->run(ball().pos, goalLine);
		
		Segment shootWindow = (_win->best) ? _win->best->segment : _win->target();
		
		//if true, then all possibilities have already been blocked
		bool noShot = !_win->best;

		//default shot line
		Segment shootLine(shootWindow.center(), ball().pos);
		
		Point windowCenter = shootWindow.center();
		
		Segment robotLine(Point(-Constants::Field::Width/2.0, Constants::Robot::Radius),
				Point(Constants::Field::Width/2.0, Constants::Robot::Radius));
		
		Point dest;
		robotLine.intersects(shootLine, &dest);
		
		Segment seg1(shootWindow.pt[0], ball().pos);
		Segment seg2(shootWindow.pt[1], ball().pos);
		
		Point pt[2];
		robotLine.intersects(seg1, &pt[0]);
		robotLine.intersects(seg2, &pt[1]);
		
		bool useCenter = noShot || (robot()->pos().nearPoint(pt[0], .25) && robot()->pos().nearPoint(pt[1], .25));
		
		if (!useCenter)
		{
			dest = pt[_index];
			
			if (dest.nearPoint(robot()->pos(), .25))
			{
				_index++;
				_index %= 2;
			}
			
			dest = pt[_index];
		}
		
		//if the ball is traveling towards the goal
		if (ball().vel.magsq() > 0.02 && ball().vel.dot(windowCenter - ball().pos) > 0)
		{
			shootLine = Segment(ball().pos, ball().pos + ball().vel.normalized() * 7.0);
			robot()->faceNone();

			if (shootLine.intersects(shootWindow))
			{
				dest = shootLine.nearestPoint(robot()->pos());
			}

			robot()->dribble(50);
		}
		else if (ball().pos.mag() < .5 && !(gameState().theirPenalty() && !gameState().playing()))
		{
			_state = Clear;
		}
		else if (closest && closest->pos().nearPoint(ball().pos, Constants::Robot::Diameter))
		{
			Point shootDir = Point::direction(closest->angle() * DegreesToRadians);

			Segment closestShootLine = Segment(closest->pos(), closest->pos() + shootDir * 7.0);

			bool closestIntersect = closestShootLine.intersects(goalLine);

			shootLine = closestShootLine;

			//dribble to catch just in case
			robot()->dribble(20);

			//if the shot will not go in the goal
			//tend toward the side it will go to
			if (!closestIntersect)
			{
				//no need to dribble yet
				robot()->dribble(0);

				//the shoot line does not intersect
				//but we still want to protect the left or right side
				Segment baseLine(Point(-Constants::Field::Width/2.0,0), Point(Constants::Field::Width/2.0,0));

				Point intersectPoint;
				if (baseLine.intersects(closestShootLine, &intersectPoint))
				{
					intersectPoint.y += Constants::Robot::Radius;

					if (intersectPoint.x > MaxX)
					{
						intersectPoint.x = MaxX;
					}
					else if (intersectPoint.x < -MaxX)
					{
						intersectPoint.x = -MaxX;
					}
				}
			}

			if (shootLine.intersects(goalLine))
			{
				dest = shootLine.nearestPoint(robot()->pos());
			}
		}
		
		//goalie should not go into the goal
		//vision can loose sight
		float margin = Constants::Ball::Radius;
		if (dest.y < Constants::Robot::Radius+margin)
		{
			dest.y = Constants::Robot::Radius+margin;
		}
		
		if (dest.x > MaxX)
		{
			dest.x = MaxX;
		}
		else if (dest.x < -MaxX)
		{
			dest.x = -MaxX;
		}
		
		robot()->move(dest);
		
		//clear the kick behavior
		_kick.assignOne(robot());
	}
	
	return true;
}
