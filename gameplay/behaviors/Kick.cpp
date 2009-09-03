#include "Kick.hpp"

#include "../Window.hpp"
#include "../Role.hpp"

#include <boost/foreach.hpp>

using namespace std;
using namespace Utils;
using namespace Geometry2d;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Kick>
        behavior("kick");

//#define DEBUG

#ifdef DEBUG
#define debug(...) fprintf(stderr, __VA_ARGS__)
#else
#define debug(...)
#endif

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay, Role *role) :
	Behavior(gameplay, role),
	target_param(this, "target"),
	mode_param(this, "mode", "auto")
{
	mode_param.legal.push_back("normal");
	mode_param.legal.push_back("auto");
	
	_win = 0;
}

Gameplay::Behaviors::Kick::~Kick()
{
	if (_win)
	{
		delete _win;
	}
}

void Gameplay::Behaviors::Kick::start()
{
	if (!_win)
	{
		_win = new WindowEvaluator(_gameplay->state());
		_win->debug = false;
	}

	_state = Intercept;
	_intercept = new Gameplay::Behaviors::Intercept(gameplay());
	_intercept->robot(robot());
	_intercept->start();
	_lastMargin = 0;
}

void Gameplay::Behaviors::Kick::run()
{
	if (!ball().valid)
	{
		// No ball
		return;
	}

	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;

	const Geometry2d::Point pos = robot()->pos();

	//goal line, for intersection detection
	Segment target(Point(-Constants::Field::GoalWidth / 2.0f, Constants::Field::Length),
			Point(Constants::Field::GoalWidth / 2.0f, Constants::Field::Length));

	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(pos);
	_win->debug = true;

	if (target_param.valid())
	{
		// Kick towards a robot
		Geometry2d::Point t = target_param.robot()->pos();
		_win->run(ballPos, t);
		_win->exclude.push_back(t);
	}
	else
	{
		// Try to kick to the goal.
		_win->run(ballPos, target);
		
		if (!_win->best && mode_param.value() == "auto")
		{
			// Use the entire end line
			target.pt[0] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
			target.pt[1] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length);
		
			_win->run(ballPos, target);
			if (!_win->best)
			{
				// Use the last 1/3rd of a side
				if (ballPos.x > 0)
				{
					target.pt[0] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
					target.pt[1] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length * 2 / 3);
				} else {
					target.pt[0] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length);
					target.pt[1] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length * 2 / 3);
				}
				_win->run(ballPos, target);
			}
		}
	}
	
	if (_win->best)
	{
		_target = _win->best->segment;
	} else {
		// Can't reach the target
		_target = _win->target();
	}

	Geometry2d::Point targetCenter = _target.center();

	int kickStrength = 255;
	
	//if kicking to a target
	//calculate kick strength
	if (target_param.valid())
	{
		const float dist = robot()->pos().distTo(targetCenter);

		const float m = robot()->state()->config.kicker.m;
		const float b = robot()->state()->config.kicker.b;

		kickStrength = int(m * dist + b);

		if (kickStrength < 0)
		{
			kickStrength = 0;
		}
		else if (kickStrength > 255)
		{
			kickStrength = 255;
		}
	}

	// Always face the ball
	robot()->face(ballPos);

	// Vector that we would shoot along if we shot now
	// assume that the ball goes where we were facing
	//Geometry2d::Point shootDir = -(pos - ballPos);
	Point shootDir = Point::direction(robot()->state()->angle * DegreesToRadians);

	//shot segment is from us in direction of shootDir
	//it is sufficiently large to handle the entire field shot
	Segment shotLine (pos, pos + shootDir * 7.0);

	//the point of intersection of the shotLine and target
	Geometry2d::Point shootPoint;

	//see if the shot line intersects where we indent to shoot
	bool intersectedTarget = shotLine.intersects(_target, &shootPoint);

	bool intersectsRobot = false;
	BOOST_FOREACH(const Packet::LogFrame::Robot& r, gameplay()->state()->opp)
	{
		//don't check against if target
		bool noAdd = (target_param.valid() && !target_param.robot()->self() && target_param.robot()->state()->shell == r.shell);
		if (r.valid && !noAdd)
		{
			if (shotLine.intersects(Circle(r.pos, Constants::Robot::Radius + Constants::Ball::Radius)))
			{
				intersectsRobot = true;
				break;
			}
		}
	}

	BOOST_FOREACH(const Packet::LogFrame::Robot& r, gameplay()->state()->self)
	{
		//don't check against if target
		bool noAdd = (target_param.valid() && target_param.robot()->self() && target_param.robot()->state()->shell == r.shell);
		if (r.valid && r.shell != robot()->state()->shell && !noAdd)
		{
			if (shotLine.intersects(Circle(r.pos, Constants::Robot::Radius + Constants::Ball::Radius)))
			{
				intersectsRobot = true;
				break;
			}
		}
	}

	// canKick is true if the robot is facing the target and the ball is between the robot and the target.
	bool canKick = intersectedTarget && robot()->state()->haveBall && !intersectsRobot && robot()->state()->radioRx.charged;

	//debug("%d ", canKick);

#if 0
	if (_state == Shoot && !canKick)
	{
		debug("Abort shoot, ball out of range\n");
		_state = Turn;
	}
#endif

	//if we already have the ball, skip approach states
	if (_state == Intercept && robot()->haveBall())
	{
		_state = Aim;
		_pivot = ballPos;
	}

	if (_state == Aim && !robot()->haveBall())
	{
		_state = Intercept;
	}

	//approach the ball at high speed using Intercept
	if (_state == Intercept)
	{
	  _intercept->target_param.set(targetCenter);
	  _intercept->run();

	  if (_intercept->done())
	    {
	      _state = Aim;
	    }
	}

	//aim towards the target
	if (_state == Aim)
	{
		debug("Aim: ");

		robot()->willKick = true;
		robot()->dribble(50);

		//robot()->pivot(_pivot, true);
	//}

#if 1
		const float clearance = Constants::Ball::Radius + Constants::Robot::Radius;

		Geometry2d::Line line(ballPos, ballPos + ballVel);
		Geometry2d::Point intercept = line.nearestPoint(pos);

		// Vectors from ball to extents of target
		Geometry2d::Point v0 = (_target.pt[0] - ballPos).normalized();
		Geometry2d::Point v1 = (_target.pt[1] - ballPos).normalized();
		float g0 = v0.angle() * RadiansToDegrees;
		float g1 = v1.angle() * RadiansToDegrees;

		if (g0 > g1)
		{
			swap(g0, g1);
		}

		float ra = robot()->state()->angle;
		float ba = (ballPos - pos).angle() * RadiansToDegrees;

		Geometry2d::Point m = ballPos + (pos - ballPos).normalized()
		        * clearance;
		if (pos.nearPoint(m, Constants::Robot::Radius))
		{
			debug("pivot  ");
			// We're close to the ideal kicking location, so fine tune by moving back and forth
			// to avoid getting stuck with a very small move.

			// From the ball's point of view:
			//     t is towards the target.
			//     s is perpendicular to that.
			Geometry2d::Point t = (targetCenter - ballPos).normalized();
			Geometry2d::Point s = t.perpCCW();


			// How far the robot is from the ball in that direction
			float d = (pos - ballPos).dot(s);


			// Pivot towards the target-ball line
			robot()->pivot(_pivot, d < 0);
		}
		else
		{
			debug("move   ");
			robot()->move(m);
		}

		if (canKick)
		{
			float margin = max(fixAngleDegrees(ra - g0), fixAngleDegrees(g1
			        - ra));

			float threshold = 0.9f * (g1 - g0) / 2;
			debug(
			        "goal %.1f, %.1f ball %.1f robot %.1f margin %.1f threshold %.1f\n",
			        g0, g1, ba, ra, margin, threshold);
			if (margin > threshold || margin <= _lastMargin)
			{
				if (ballPos.nearPoint(pos, clearance + Constants::Robot::Radius))
				{
					debug("Shoot\n");
					_shootStart = pos;
					_shootMove = pos + (ballPos - pos).normalized() * 0.2f;
					_shootBallStart = ballPos;
					_state = Shoot;
				}
			}

			_lastMargin = margin;
		}
		else
		{
			debug("wait %.1f %.1f %.1f %.1f", g0, g1, ba, ra);
			_lastMargin = 0;
		}
		debug("\n");
#endif
	}

	if (_state == Shoot)
	{
		debug("Shoot: %f", ballVel.dot(pos - ballPos));
		//robot()->kick(strength_param.value());
		robot()->kick(kickStrength);
		robot()->face(targetCenter);
		robot()->dribble(0);
		robot()->move(_shootMove);

		// If the robot has moved more than half a meter since starting to shoot, we are probably
		// just pushing the ball around, so give up.
		if (!ballPos.nearPoint(_shootBallStart, 0.2f) || !pos.nearPoint(_shootStart, 0.5f))
		{
			// Ball was kicked or we moved far enough to give up
			_state = Done;
			debug("  done");
		}
		debug("\n");
	}
}

bool Gameplay::Behaviors::Kick::done()
{
	return _state == Done;
}

float Gameplay::Behaviors::Kick::score(Robot* robot)
{
	return (robot->pos() - ball().pos).magsq();
}
