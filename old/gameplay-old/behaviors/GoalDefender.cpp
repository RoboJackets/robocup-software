#include "GoalDefender.hpp"



Gameplay::Behaviors::GoalDefender::GoalDefender(GameplayModule *gameplay):
    Behavior(gameplay)
{
}

bool Gameplay::Behaviors::GoalDefender::run()
{
	if (!assigned() || !allVisible())
	{
		return false;
	}

	Geometry2d::Point ballFuture = ball().pos + ball().vel*0.1;
	const float radius = .7;

	//goal line, for intersection detection
	Geometry2d::Segment goalLine(Geometry2d::Point(-Constants::Field::GoalWidth / 2.0f, 0),
								  Geometry2d::Point(Constants::Field::GoalWidth / 2.0f, 0));

	// Update the target window
	_winEval->exclude.clear();
	for (Robot *r :  _robots)
	{
		_winEval->exclude.push_back(r->pos());
	}

	_winEval->run(ballFuture, goalLine);

	Window* best = 0;
	float bestDist = 0;

	// finds the closest segment to the ball
	for (Window* window :  _winEval->windows)
	{
		Geometry2d::Segment seg(window->segment.center(), ball().pos);
		float newDist = seg.distTo(Behavior::robot()->pos());

		if (seg.length() > Constants::Robot::Radius && (!best || newDist < bestDist))
		{
			best = window;
			bestDist = newDist;
		}
	}

	if (!best)
	{
	    //FIXME - What if there are no windows?
	    return true;
	}

	Geometry2d::Circle arc(Geometry2d::Point(), radius);
	Geometry2d::Line ballTravel(ball().pos, ballFuture);
	Geometry2d::Point dest[2];
	bool ballTravelIntersects = ballTravel.intersects(arc, &dest[0], &dest[1]);
	Geometry2d::Point blockPoint = (dest[0].y > 0 ? dest[0] : dest[1]);
	bool movingTowardsGoal = ballFuture.mag() < ball().pos.mag();

	if(ballTravelIntersects && blockPoint.y > 0 && movingTowardsGoal)
	{
		// If ball is traveling towards the goal, block it.
		state()->drawText("YY", blockPoint, Qt::magenta);
	}
	else
	{	// Stand in the largest open window
		Geometry2d::Line bestWindowLine(best->segment.center(), Geometry2d::Point(0,0));
		Geometry2d::Point blockPoint = (best->segment.center().normalized()) * radius;
		state()->drawText("XX", blockPoint, Qt::white);
	}



	/*
	_winEval->exclude.push_back(Behavior::robot()->pos());

	//exclude robots that arn't the defender
	//_winEval->run(ball().pos, goalLine);

	for (Defender *f :  otherDefenders)
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
		for (Window* window :  _winEval->windows)
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
		for (Window* window :  _winEval->windows)
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

	if (best)
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
	}*/

	return false;
}

bool Gameplay::Behaviors::GoalDefender::assign(std::set<Robot *> &available)
{
	_robots.clear();
	if (!takeBest(available))
	{
	    return false;
	}

	//initialize windowevaluator
	_winEval = new Gameplay::WindowEvaluator(Behavior::gameplay()->state());
	_winEval->debug = false;

	return true;
}

bool Gameplay::Behaviors::GoalDefender::done()
{
	return false;
}

float Gameplay::Behaviors::GoalDefender::score(Robot* robot)
{
	// selects the robot closest to our goal.
	return robot->pos().magsq();
}
