// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

//FIXME - The rules allow for changing the goalie only in certain circumstances.  Make sure we do this right.

#include "Goalie.hpp"

#include <gameplay/Play.hpp>
#include <gameplay/Window.hpp>
#include <Geometry2d/util.h>

#include <boost/foreach.hpp>

#include <Constants.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

static const float MaxX = Field_GoalWidth / 2.0f;
static const float margin = /*Ball_Radius * 4*/ 0;

void Gameplay::Behaviors::Goalie::createConfiguration(Configuration *cfg) {

}

Gameplay::Behaviors::Goalie::Goalie(GameplayModule *gameplay) :
				SingleRobotBehavior(gameplay), _kick(gameplay) {
	_win = new WindowEvaluator(_gameplay->state());
	_win->debug = true;

	_state = Defend;
	robot = 0;
	_index = 0;

}

Gameplay::Behaviors::Goalie::~Goalie() {
	delete _win;
}

/** Checks whether or not the given ball is in the defense area. */
bool ballIsInGoalieBox(Ball ball) {
	if (abs(ball.pos.x) < Field_GoalFlat / 2.0f) // Ball is in center (rectangular) portion of defensive bubble
	{
		if (ball.pos.y > 0 && ball.pos.y < Field_ArcRadius) {
			return true;
		} else {
			return false;
		}
	} else if (abs(ball.pos.x) < (Field_ArcRadius + Field_GoalFlat / 2.0f)) // Ball is in one of the side (arc) portions of defensive bubble
	{
		double adjusted_x = abs(ball.pos.x) - (Field_GoalFlat / 2.0f);
		double max_y = sqrt( (Field_ArcRadius * Field_ArcRadius) - (adjusted_x * adjusted_x));
		if (ball.pos.y > 0 && ball.pos.y <= max_y) {
			return true;
		} else {
			return false;
		}
	}
	return false;
}

void Gameplay::Behaviors::Goalie::assign(set<OurRobot *> &available) {
	if (!robot) {
		// Take the robot nearest the goal
		assignNearest(robot, available, Geometry2d::Point());
		if (robot) {
			printf("Goalie: no robot, took %d\n", robot->shell());
		} else {
			// 			printf("Goalie: not assigned\n");
		}
		/*	} else if (robot && !robot->visible)
		{
		 //FIXME - Goalie replacement
		 available.erase(robot);*/
	} else {
		// Keep the current goalie, and prevent it from being used in a play
		available.erase(robot);
	}
}

bool Gameplay::Behaviors::Goalie::opponentsHavePossession() {
	if(opponentWithBall())
		return true;
	else
		return false;
}

float angleBetweenThreePoints(Point a, Point b, Point vertex)
{
	float VA = sqrt(powf(vertex.x-a.x,2) + powf(vertex.y-a.y,2));
	float VB = sqrt(powf(vertex.x-b.x,2) + powf(vertex.y-b.y,2));
	float AB = sqrt(powf(a.x-b.x,2) + powf(a.y-b.y,2));
	return acosf((VA*VA + VB*VB - AB*AB)/(2*VA*VB));
}

Robot* Gameplay::Behaviors::Goalie::opponentWithBall()
{
	Robot* closest = 0;
	float bestDist = 0;
	BOOST_FOREACH(Robot *r, state()->opp){
		float dist = r->pos.distTo(ball().pos);
		if (!closest || dist < bestDist)
		{
			closest = r;
			bestDist = dist;
		}
	}
	float angle = 3.14 * (closest->angle / 180);
	float theta = angleBetweenThreePoints(Point(closest->pos.x + cos(angle), closest->pos.y + sin(angle)),ball().pos,closest->pos);
	float maxRadius = Robot_Diameter + Robot_Diameter*cos(theta);
	//std::cout << theta << std::endl;
	if(closest && closest->pos.nearPoint(ball().pos, maxRadius))
		return closest;
	else
		return 0;
}

bool Gameplay::Behaviors::Goalie::ballIsMovingTowardsGoal()
{
	Segment goalLine(Point(-MaxX, 0), Point(MaxX, 0));
	Segment ballPath(ball().pos, (ball().pos + 5*ball().vel.normalized()));
	//if the ball is traveling towards the goal
	return (ball().vel.magsq() > 0.02 && ballPath.intersects(goalLine));
}

bool Gameplay::Behaviors::Goalie::run()
{
	if (!robot || !robot->visible)
	{
		return true;
	}

	_kick.robot = robot;

	if(!ball().valid)
		_state = None;
	else if(gameState().theirPenalty() && !gameState().playing())
		_state = SetupPenalty;
	else if(ballIsInGoalieBox(ball()))
		_state = Clear;
	else if (ballIsMovingTowardsGoal())
			_state=Intercept;
	else if (opponentsHavePossession())
		_state = Block;
	else
		_state = Defend;


	//robot->addText(QString("Opp: %1").arg(this->opponentWithBall()->shell));

	robot->face(ball().pos);

	switch (_state)
	{

	case Defend:
	{
		robot->addText(QString("State: Defend"));

		float destx = ( ball().pos.x / Field_Width ) * MaxX;

		robot->move(Point(destx, Robot_Radius), true);

	}
	break;

	case Block:
	{
		robot->addText(QString("State: Block"));

		//Robot* opposingKicker = opponentWithBall();
		//Line shotLine = Line(opposingKicker->pos, ball().pos);

		Line shotLine = Line(ball().pos, ball().pos + ball().vel.normalized());

		Circle blockCircle = Circle(Point(0,0), Field_GoalWidth/2.0f);

		Point dest;
		if(blockCircle.intersects(shotLine, &dest)) {
			// shotLine intersects blockCircle
			robot->move(dest, true);
		} else {
			// shotLine does not intersect blockCircle
			Line blockLine = Line(Point(-MaxX,Robot_Radius), Point(MaxX,Robot_Radius));
			blockLine.intersects(shotLine, &dest);
			dest.x = min(MaxX, dest.x);
			dest.x = max(-MaxX, dest.x);
			robot->move(dest, true);
		}
	}
	break;

	case Intercept:
	{
		robot->addText(QString("State: Intercept"));

		robot->face(ball().pos, true);
		Line ballPath(ball().pos, (ball().pos + ball().vel.normalized()));
		Point dest; //destination point- where robot needs to move to
		dest = ballPath.nearestPoint(robot->pos);
		robot->move(dest, true);
	}
	break;

	case Clear:
	{
		robot->addText(QString("State: Clear"));
		//Ball is in defense area, get it out of there.
		robot->dribble(50);

//		Segment opponentGoal(Point(-MaxX, Field_Length), Point(MaxX, Field_Length));
//		_win->clear();
//		_win->run(ball().pos, opponentGoal);
//		if(_win->best()) {
//			_kick.setTarget(_win->best()->segment);
//		} else {
//			//TODO pass to one of our players
//			_kick.setTargetGoal();
//		}

		_kick.enableGoalLineShot = true;
		_kick.enableLeftDownfieldShot = true;
		_kick.enableRightDownfieldShot = true;

		_kick.run();
	}
	break;

	case SetupPenalty:
	{
		robot->addText(QString("State: setupPenalty"));
		// Touch the goal line, as required by the rules, and move to block the shot.
		Robot *penaltyKicker = 0;
		float smallestDist = 0;
		BOOST_FOREACH(Robot *r, state()->opp)
		{
			float dist = r->pos.distTo(Point(0,Field_PenaltyDist));
			if(!penaltyKicker || dist < smallestDist)
			{
				penaltyKicker = r;
				smallestDist = dist;
			}
		}
		float angle = 3.14 * (penaltyKicker->angle / 180);
		Line shotLine = Line(penaltyKicker->pos, Point(penaltyKicker->pos.x + cos(angle), penaltyKicker->pos.y + sin(angle)));
		Segment robotSegment = Segment(Point(-MaxX,Robot_Radius), Point(MaxX,Robot_Radius));
		Point dest;

		if(shotLine.intersects(robotSegment, &dest))
		{
			dest.x = max(-MaxX + Robot_Radius, dest.x);
			dest.x = min( MaxX - Robot_Radius, dest.x);
			robot->move(dest, true);
		} else {
			robot->move(Point(0, Robot_Radius), true);
		}
	}
	break;

	case None:
	{
		robot->addText(QString("State: None"));
		if(abs(robot->pos.x) > Field_GoalWidth/2.0f || robot->pos.y > Robot_Radius + margin) {
			robot->move(Geometry2d::Point(0, Robot_Radius));
		}
	}
	break;
	}
	//clear the kick behavior
	_kick.restart();

// ---------------------------------------------------------------------------------------------------------------------------------

	/*	if (!ball().valid) {
		//FIXME - Stay where we are, if we are near the goal (done @matt)
		if(abs(robot->pos.x) > Field_GoalWidth/2.0f || robot->pos.y > Robot_Radius + margin) {
			robot->move(Geometry2d::Point(0, Robot_Radius));
		}
		return true;
	}

	robot->addText(QString("state %1").arg(_state));
	//if the ball is in the defense area, clear it
	if (_state == Clear) {
		robot->dribble(50);

		bool done = !_kick.run();

		//If I've finished kicking, the ball rolled out of the goalie box, or the ball went into the goal, end "Clear" state
		if (done || !ballIsInGoalieBox(ball())) {
			_state = Defend;
		}
	} else if (_state == Defend) {
		robot->face(ball().pos);

		Robot* closest = 0;
		float bestDist = 0;
		BOOST_FOREACH(Robot *r, state()->opp){
			float dist = r->pos.distTo(ball().pos);
			if (!closest || dist < bestDist)
			{
				closest = r;
				bestDist = dist;
			}
		}

		//const Point goalBack(0,-Field_GoalDepth);

		//goal line, for intersection detection
		Segment goalLine(Point(-MaxX, 0), Point(MaxX, 0));

		// Pick the largest available window between the ball and our goal, ignoring our goalie as an obstacle.
		_win->exclude.clear();
		_win->exclude.push_back(robot->pos);
		_win->run(ball().pos, goalLine);
		Segment shootWindow = (_win->best()) ? _win->best()->segment : _win->target();

		bool noShot = !_win->best(); // If true, then all possibilities have already been blocked

		//default shot line
		Segment shootLine(shootWindow.center(), ball().pos);

		Point windowCenter = shootWindow.center();

		Segment robotLine(Point(-Field_Width / 2.0, Robot_Radius), Point(Field_Width / 2.0, Robot_Radius));

		Point dest;
		robotLine.intersects(shootLine, &dest);

		Segment seg1(shootWindow.pt[0], ball().pos);
		Segment seg2(shootWindow.pt[1], ball().pos);

		Point pt[2];
		robotLine.intersects(seg1, &pt[0]);
		robotLine.intersects(seg2, &pt[1]);

		bool useCenter = noShot || (robot->pos.nearPoint(pt[0], .25) && robot->pos.nearPoint(pt[1], .25));
		bool setupPenalty = gameState().theirPenalty() && !gameState().playing();
		useCenter |= setupPenalty;

		if (!useCenter) {
			dest = pt[_index];

			if (dest.nearPoint(robot->pos, .25)) {
				_index++;
				_index %= 2;
			}

			dest = pt[_index];
			robot->addText("useCenter");
		}

		if (!setupPenalty) {
			//if the ball is traveling towards the goal
			if (ball().vel.magsq() > 0.02 && ball().vel.dot(windowCenter - ball().pos) > 0) {
				shootLine = Segment(ball().pos, ball().pos + ball().vel.normalized() * 7.0);
				robot->faceNone();


				if (shootLine.intersects(shootWindow)) {
					robot->addText("case 1");
					//dest = shootLine.nearestPoint(robot->pos);
					// Move forward along the shotLine to block a narrower window.
					Circle circle = Circle(windowCenter, 0.5f);
					circle.intersects(shootLine, &dest);
				}

				robot->dribble(50);

			} else if (ballIsInGoalieBox(ball())) {
				_state = Clear;

			} else if (closest && closest->pos.nearPoint(ball().pos, Robot_Diameter)) {
				Point shootDir = Point::direction( closest->angle * DegreesToRadians);

				Segment closestShootLine = Segment(closest->pos, closest->pos + shootDir * 7.0);

				bool closestIntersect = closestShootLine.intersects(goalLine);

				shootLine = closestShootLine;

				//dribble to catch just in case
				robot->dribble(20);

				//if the shot will not go in the goal
				//tend toward the side it will go to
				if (!closestIntersect) {
					//no need to dribble yet
					robot->dribble(0);

					//the shoot line does not intersect
					//but we still want to protect the left or right side
					Segment baseLine(Point(-Field_Width / 2.0, 0), Point(Field_Width / 2.0, 0));

					Point intersectPoint;
					if (baseLine.intersects(closestShootLine, &intersectPoint)) {
						intersectPoint.y += Robot_Radius;

						if (intersectPoint.x > MaxX) {
							intersectPoint.x = MaxX;
						} else if (intersectPoint.x < -MaxX) {
							intersectPoint.x = -MaxX;
						}
					}
				}

				if (shootLine.intersects(goalLine)) {
					robot->addText("case 2");
					dest = shootLine.nearestPoint(robot->pos);
				}
			}
		}

		//goalie should not go into the goal
		//vision can loose sight
		if (dest.y < Robot_Radius + margin) {
			robot->addText("case 3");
			dest.y = Robot_Radius + margin;
		}

		if (dest.x > MaxX) {
			robot->addText("case 4");
			dest.x = MaxX;
		} else if (dest.x < -MaxX) {
			robot->addText("case 5");
			dest.x = -MaxX;
		}

		state()->drawLine(robot->pos, dest, Qt::white);
		robot->move(dest);

		//clear the kick behavior
		_kick.restart();
	}
	 */
	return true;
}
