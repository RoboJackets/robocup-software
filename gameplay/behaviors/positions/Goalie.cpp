// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "Goalie.hpp"

#include "../../Role.hpp"
#include "../../Window.hpp"

#include <LogFrame.hpp>
#include <boost/foreach.hpp>

using namespace Geometry2d;

static const float MaxX = Constants::Field::GoalWidth / 2.0f;

Gameplay::Behaviors::Goalie::Goalie(GameplayModule *gameplay, Role *role):
	Behavior(gameplay, role),
	_kick(gameplay, role)
{
	_name = "Goalie";
	_win = 0;
	
	_state = Defend;
}

Gameplay::Behaviors::Goalie::~Goalie()
{
	delete _win;
}

void Gameplay::Behaviors::Goalie::start()
{
	if (!_win)
	{
		_win = new WindowEvaluator(_gameplay->state());
		_win->debug = true;
	}
	
	_kick.robot(robot());
	_kick.start();
	
	_state = Defend;
	
	_index = 0;
}

void Gameplay::Behaviors::Goalie::run()
{
	Packet::LogFrame::Robot* self = robot()->state();

	if (!ball().valid)
	{
		return;
	}
	
	const Packet::LogFrame::Ball& ball = gameplay()->state()->ball;
	
	if (_state == Clear)
	{
		//if the ball is in the defense area...clear it
		_kick.target_param.clear();
		_kick.mode_param.set("auto");
		
		robot()->dribble(50);
		
		_kick.run();
		
		//done or give up
		//if ball in goal...don't go in goal
		if (_kick.done() || ball.pos.mag() > .75 || ball.pos.y < 0)
		{
			_state = Defend;
		}
	}
	
	if (_state == Defend)
	{
		self->cmd.goalOrientation = ball.pos;
		self->cmd.face = Packet::LogFrame::MotionCmd::Endpoint;

		Packet::LogFrame::Robot* closest = 0;
		float bestDist = 0;
		BOOST_FOREACH(Packet::LogFrame::Robot& r, gameplay()->state()->opp)
		{
			float dist = r.pos.distTo(ball.pos);
			if (!closest || dist < bestDist)
			{
				closest = &r;
				bestDist = dist;
			}
		}

		//const Point goalBack(0,-Constants::Field::GoalDepth);

		//goal line, for intersection detection
		Segment goalLine(Point(-MaxX, 0), Point(MaxX, 0));
		
		// Update the target window
		_win->exclude.clear();
		_win->exclude.push_back(self->pos);
		_win->run(ball.pos, goalLine);
		
		Segment shootWindow = (_win->best) ? _win->best->segment : _win->target();
		
		//if true, then all possibilities have already been blocked
		bool noShot = !_win->best;

		//default shot line
		Segment shootLine(shootWindow.center(), ball.pos);
		
		Point windowCenter = shootWindow.center();
		
		Segment robotLine(Point(-Constants::Field::Width/2.0, Constants::Robot::Radius),
				Point(Constants::Field::Width/2.0, Constants::Robot::Radius));
		
		Point dest;
		robotLine.intersects(shootLine, &dest);
		
		Segment seg1(shootWindow.pt[0], ball.pos);
		Segment seg2(shootWindow.pt[1], ball.pos);
		
		Point pt[2];
		robotLine.intersects(seg1, &pt[0]);
		robotLine.intersects(seg2, &pt[1]);
		
		bool useCenter = noShot || (self->pos.nearPoint(pt[0], .25) && 
			self->pos.nearPoint(pt[1], .25));
		
		if (!useCenter)
		{
			dest = pt[_index];
			
			if (dest.nearPoint(self->pos, .25))
			{
				_index++;
				_index %= 2;
			}
			
			dest = pt[_index];
		}
		
		//if the ball is traveling towards the goal
		if (ball.vel.magsq() > 0.02 && ball.vel.dot(windowCenter - ball.pos) > 0)
		{
			shootLine = Segment(ball.pos, ball.pos + ball.vel.normalized() * 7.0);
			self->cmd.face = Packet::LogFrame::MotionCmd::None;

			if (shootLine.intersects(shootWindow))
			{
				dest = shootLine.nearestPoint(self->pos);
			}

			robot()->dribble(50);
		}
		else if (ball.pos.mag() < .5)
		{
			_state = Clear;
		}
		else if (closest && closest->pos.nearPoint(ball.pos, Constants::Robot::Diameter))
		{
			Point shootDir = Point::direction(closest->angle * DegreesToRadians);

			Segment closestShootLine = Segment(closest->pos, closest->pos + shootDir * 7.0);

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
				dest = shootLine.nearestPoint(self->pos);
			}
		}
		
		//goalie should not go into the goal
		//vision can loose sight
		if (dest.y < Constants::Robot::Radius)
		{
			dest.y = Constants::Robot::Radius;
		}
		
		if (dest.x > MaxX)
		{
			dest.x = MaxX;
		}
		else if (dest.x < -MaxX)
		{
			dest.x = -MaxX;
		}
	
		self->cmd.goalPosition = dest;
		
		//clear the kick behavior
		_kick.start();
	}
}
