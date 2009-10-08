#include "ReceivePass.hpp"
#include "Kick.hpp"

using namespace std;
using namespace Utils;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::ReceivePass> behavior("receive_pass");

Gameplay::Behaviors::ReceivePass::ReceivePass(GameplayModule *gameplay, Role *role):
	Behavior(gameplay, role),
	range_param(this, "range", 0.3),
	target_param(this, "target")
{
	_kick = new Gameplay::Behaviors::Kick(gameplay);
	start();
}

Gameplay::Behaviors::ReceivePass::~ReceivePass()
{
	delete _kick;
}

void Gameplay::Behaviors::ReceivePass::start()
{
	_kick->robot(robot());
	_kick->target_param.set(target_param.robot());
	
	_state = Wait;
	_waitCount = 0;
}

void Gameplay::Behaviors::ReceivePass::run()
{
	if (!ball().valid)
	{
		// No ball
		return;
	}
	
	Geometry2d::Point ballPos = ball().pos;
	Geometry2d::Point ballVel = ball().vel;
	
	// Always face the ball
	robot()->face(ballPos);
	
	Geometry2d::Point pos = robot()->pos();
	
	const float clearance = Constants::Ball::Radius + Constants::Robot::Radius;
	
	float ballSpeed = ballVel.mag();
	Geometry2d::Line line(ballPos, ballPos + ballVel);
	Geometry2d::Point intercept = line.nearestPoint(pos);
	
	if (_state == Receive)
	{
        if (ballPos.nearPoint(pos, Constants::Robot::Radius + Constants::Ball::Radius + 0.1f))
        {
            // Close enough
            _state = Kick;
        } else printf("robot to ball %f\n", (ballPos - pos).mag());
	}
	
	if (_state == Wait)
	{
		bool toward = ballVel.dot(pos - ballPos) > 0.1f;
		if (toward && pos.nearPoint(intercept, range_param.value()))
		{
			// The ball has been kicked and we can intercept it
			if (_waitCount < 5)
			{
				++_waitCount;
			} else {
				_state = Receive;
			}
		} else {
			_waitCount = 0;
		}
	} else if (_state == Receive)
	{
		bool slow = false;
		Geometry2d::Point ballAccel = ball().accel;
		if (ballSpeed < 0.2f)
		{
			slow = true;
		} else if (ballVel.dot(pos - ballPos) > 0 && ballAccel.dot(ballVel) < 0)
		{
			// Ball is moving towards us and slowing down
			float a = ballAccel.mag();
			
			// Find the time at which the velocity will be zero if the acceleration is in the
			// same direction as the velocity.
			float t = ballSpeed / a;
			
			// Find the point at which the ball will stop (if accel is in the direction of velocity).
			// If acceleration is not directly against velocity, the ball may never stop, but we're
			// just approximating.  Generally the ball doesn't curve.
			Geometry2d::Point stop = ballPos + ballVel * t + ballAccel * 0.5f * t * t;
			
			float q = (stop - pos).dot(ballPos - pos);
			printf("stop at %f %f\n", t, q);
			
			if (q > 0)
			{
				// Stopping point is on the same side of the robot as the ball,
				// which means the ball will stop before it passes us and we need to go get it.
				slow = true;
			}
		}
		
		if (slow)
		{
			printf("slow\n");
			// Ball is too slow.  Go get it.
//			robot()->move(ballPos - (target - ballPos).normalized() * (clearance + 0.05f));
			robot()->move(ballPos + (pos - ballPos).normalized() * (clearance + 0.05f));
		} else if ((intercept - ballPos).dot(ballVel) < 0 && ballSpeed > 0.05f)
		{
			//FIXME - I don't like this part.  Redo it.
			printf("behind\n");
			// The ball is moving away from (has already passed) the intercept point.
			// Chase after the ball and try to get in front of it.
			float distance = min(1.0f, (clearance + 0.05f) + ballSpeed / 2);
			intercept = ballPos + ballVel / ballSpeed * distance;
			
			if ((intercept - ballPos).magsq() > (pos - ballPos).magsq())
			{
				// The new point would move us away from the ball.
				// Stay where we are.
//					return;
			}
			robot()->move(intercept);
		} else {
			// Get in the way of the ball
			printf("intercept\n");
			robot()->move(intercept);
		}
	} else if (_state == Kick)
	{
		_kick->run();
	}
}

bool Gameplay::Behaviors::ReceivePass::done()
{
	return _state == Kick && _kick->done();
}
