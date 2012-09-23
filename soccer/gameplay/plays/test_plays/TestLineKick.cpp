#include "TestLineKick.hpp"

#include <Utils.hpp>
#include <stdio.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays:: TestLineKick, "Test")

static const double robot_w = 2 * M_PI * 2;
static const double test_w = 2 * M_PI * 0.5;

static const double WAIT_TIME = 5;

Gameplay::Plays:: TestLineKick:: TestLineKick(GameplayModule *gameplay):
	Play(gameplay),
	_LineKick(gameplay)
{
	_time = 0;
	_lastKickTime = 0;
	_state = State_Ready;
	_kickPower=255;
	//_file.open(("Kick_Exp_-" + QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log").toStdString()).c_str(), fstream::out);
	//_file << str(format("%f %f 0 0 0 0\n") % test_w % robot_w);

	_robotPos = Geometry2d::Point(0,1);
}

bool Gameplay::Plays:: TestLineKick::run()
{
	if (_gameplay->playRobots().empty())
	{
		return false;
	}

	if (_time == 0)
	{
		_time = state()->timestamp;
		return true;
	}

	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot || !robot->visible)
	{
		return false;
	}

	_LineKick.robot = robot;


	if    (_state==State_Ready)
		robot->addText(QString("Ready!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
	else if(_state==State_Wait)
		robot->addText(QString("Wait!!!!!!!!!!!!!!!!!!!!!!!!!!"));
	else  if(_state==State_Kick)
			robot->addText(QString("Kick!!!!!!!!!!!!!!!!!!!!!!!!!!"));




	double curTime = (state()->timestamp - _time) / 1000000.0;


	if (_state == State_Ready)
		{
		robot->move(_robotPos, true);
		robot->face(Geometry2d::Point(0,10));
		if( curTime-_lastKickTime < WAIT_TIME )
					_state=State_Ready;
				else
				{
			     if(robot->pos.distTo(_robotPos) < 0.01)
					_state = State_Kick;
				}


		}

	else if(_state==State_Kick)

					{
		              _LineKick.run();

						   if(_LineKick.done())
						    		{
							       _lastKickTime=curTime;
						    		_state = State_Wait;
						    		}

						    	else // ball off ground

						    		_state = State_Kick;

	                }

	else if(_state==State_Wait)
	{


	        if(!ball().valid )
				_state=State_Wait;
			else
				_LineKick.restart();
				_state = State_Ready;





	}



	return true;
}
