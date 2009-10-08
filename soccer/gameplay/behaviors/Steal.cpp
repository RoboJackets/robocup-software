#include "Steal.hpp"

#include "../Role.hpp"
#include <LogFrame.hpp>

using namespace std;
using namespace Utils;
using namespace Geometry2d;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Steal>
        behavior("steal");

#define DEBUG

#ifdef DEBUG
#define debug(...) fprintf(stderr, __VA_ARGS__)
#else
#define debug(...)
#endif

Gameplay::Behaviors::Steal::Steal(GameplayModule *gameplay, Role *role) :
	Behavior(gameplay, role),
	_intercept(gameplay, role)
{
}

void Gameplay::Behaviors::Steal::start()
{
	_state = Intercept;
	//_intercept->robot(robot());
	//_intercept->start();
	_lastMargin = 0;
}

void Gameplay::Behaviors::Steal::run()
{
#if 0
	if (!ball().valid)
	{
		// No ball
		return;
	}
	
	const Geometry2d::Point pos = robot()->pos();
	
	Geometry2d::Point target =  target_param.robot()->pos();
	
	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;
	
	// Always face the ball
	robot()->face(ballPos);
	

	//Intercept the ball and shooter
	if (_state == Intercept)
	  {
	  _intercept->target_param.set(target);
	  _intercept->run();

	  if (_intercept->done())
	    {
	      _state = Maneuver;
	    }
	  }

	//maneuver so ball is between opp and robot
	if (_state == Maneuver)
	{
	  robot()->willKick = true;
	  
	  //close to ball
	  robot()->move(ballPos);
	  
	  if (robot()->pos().nearPoint(ballPos, 
				       1.1*(Constants::Robot::Radius + Constants::Ball::Radius)))
	    {
	      _state = Stealing;
	    }
	}
	
	if (_state == Stealing)
	{
	  //spin to steal the ball
	  Packet::LogFrame::MotionCmd::SpinType dir = Packet::LogFrame::MotionCmd::SpinCCW;
	  robot()->spin(dir);
	  if (target.distTo(ballPos) > 
	      (Constants::Robot::Radius + 
	       Constants::Ball::Radius)*1.5)
	  {
	    //ball was stolen successfully
	    _state = Done;
	    dir = Packet::LogFrame::MotionCmd::NoSpin;
	    robot()->spin(dir);
	  }
	  else if (robot()->pos().distTo(ballPos) > (Constants::Robot::Radius + Constants::Ball::Radius)*1.1
		   || robot()->pos().distTo(target) > (Constants::Robot::Radius + Constants::Ball::Radius)*1.1)
	    {
	      //need to reacquire
	      dir = Packet::LogFrame::MotionCmd::NoSpin;
	      robot()->spin(dir);
	      _state = Intercept;
	      _intercept->start();
	    }
	}
	
	debug("\n");
#endif
}

bool Gameplay::Behaviors::Steal::done()
{
	return _state == Done;
}

float Gameplay::Behaviors::Steal::score(Robot* robot)
{
	return (robot->pos() - ball().pos).magsq();
}
