#include "Fullback.hpp"

#include <gameplay/behaviors/positions/Goalie.hpp>
#include <Constants.hpp>
#include <gameplay/Window.hpp>
#include <Geometry2d/util.h>

#include <vector>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace std;

void Gameplay::Behaviors::Fullback::createConfiguration(Configuration *cfg)
{

}

Gameplay::Behaviors::Fullback::Fullback(GameplayModule *gameplay, Side side):
	Behavior(gameplay),
	_winEval(gameplay->state()),
	_side(side)
{
	robot = 0;
	_state = Init;

	_winEval.debug = false;
}

bool Gameplay::Behaviors::Fullback::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// Do not avoid opponents when planning while we are close to the goal
	const float oppAvoidThresh = 2.0; // meters radius of goal
	if (robot->pos.nearPoint(Geometry2d::Point(), oppAvoidThresh))
		robot->avoidOpponents(false);
	else
		robot->avoidOpponents(true);

	// we multiply by 0.3 here to look 0.3s into the future when considering the ball position
	// also, this will be used for where the robots will face
	Geometry2d::Point ballFuture = ball().pos + ball().vel*0.3;

	//goal line, for intersection detection
	Geometry2d::Segment goalLine(Geometry2d::Point(-Field_GoalWidth / 2.0f, 0),
								  Geometry2d::Point(Field_GoalWidth / 2.0f, 0));

	// Update the target window
	_winEval.exclude.clear();
	_winEval.exclude.push_back(robot->pos);
	
	//exclude robots that arn't the fullback
	//_winEval.run(ball().pos, goalLine);
	
	BOOST_FOREACH(Fullback *f, otherFullbacks)
	{
		if (f->robot)
		{
			_winEval.exclude.push_back(f->robot->pos);
		}
	}
	
	_winEval.run(ballFuture, goalLine);
	
	Window* best = 0;

	bool needTask = false;
	
	//pick biggest window on appropriate side
	Goalie* goalie = _gameplay->goalie();
	if (goalie && goalie->robot && _side != Center)
	{
		BOOST_FOREACH(Window* window, _winEval.windows)
		{
			if (_side == Left)
			{
				if (!best || window->segment.center().x < goalie->robot->pos.x)
				{
					best = window;
				}
			}
			else if (_side == Right)
			{
				if (!best || window->segment.center().x > goalie->robot->pos.x)
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
		BOOST_FOREACH(Window* window, _winEval.windows)
		{
			Geometry2d::Segment seg(window->segment.center(), ball().pos);
			float newDist = seg.distTo(robot->pos);
			
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
			robot->move(shootLine.nearestPoint(robot->pos));
			robot->faceNone();
		}
		else
		{
			const float winSize = winSeg.length();
				
			if (winSize < Ball_Radius)
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
						robot->move(dest[0]);
					}
					else
					{
						robot->move(dest[1]);
					}
					//robot->face(ballFuture);
					// Using regular pos rather than future because velocity approx on ball is not exact
					// enough and this leads to the robots turning backwards, towards the goal, when the
					// ball is shot at the goal.
					robot->face(ball().pos);
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
		robot->face(ball().pos, true);
	}

	// Turn dribbler on when ball is near
	if(ball().pos.y < Field_Length / 2)
	{
		robot->dribble(255);
	}

	// If ball sensor is tripped and we are not facing towards the goal, fire.
	Geometry2d::Point backVec(1,0);
	Geometry2d::Point backPos(-Field_Width/2,0);
	Geometry2d::Point shotVec(ball().pos - robot->pos);
	Geometry2d::Point shotPos(robot->pos);
	Geometry2d::Point backVecRot(backVec.perpCCW());
	bool facingBackLine = (backVecRot.dot(shotVec) < 0);
	if(!facingBackLine)
	{
		robot->kick(255);
	}

	return false;
}
