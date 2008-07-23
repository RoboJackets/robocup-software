#include "MotionController.hpp"

#include <QUdpSocket>
#include <QTimer>
#include <QMutex>
#include <QWaitCondition>
#include <QMutexLocker>

#include <Packet/PacketSender.hpp>
#include <Packet/CommData.hpp>
#include <Packet/LogMotion.hpp>
#include <Packet/PacketReceiver.hpp>

#include <Geometry/Point2d.hpp>
#include <Geometry/Circle2d.hpp>
#include <Geometry/Segment.hpp>

#include <Sizes.h>

#include <math.h>

#include "SimplePlanner.hpp"

using namespace Packet;
using namespace Geometry;

MotionController::MotionController(Team t, ConfigFile& cfg, bool autoOn)
	: QThread(), _team(t), _controlState(Auto), _runState(Running), 
	_running(true), _cfg(cfg)
{
	const char* team = "Yellow";
	if (_team == Blue)
	{
		team = "Blue";
	}
	printf("New Motion Controller for %s.\n", team);
	
	_newVision = false;
	
	//by default we want to be running, until we get a ref command
	_ref.state = Ref::Running;
	
	//initialize the robots
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_robots[i] = new Robot(cfg.robotConfig(i), 
				_visionData, 
				_commData.robots[i], 
				_logData.robots[i]);
		
		_robots[i]->newPathPlanner<SimplePlanner>();
	}
	
	try
	{
		_input = new InputHandler(_cfg);
		connect(_input, SIGNAL(playPauseButton()), this, SLOT(playPause()));
		connect(_input, SIGNAL(manualAutoButton()), this, SLOT(autoMan()));
		
		_input->start();

		printf("Using gamepad 'js0' for input\n");
	}
	catch (std::runtime_error re)
	{
		_input = 0;
		printf("No Controller Input [%s]. Always Auto\n", re.what());
	}
	
	if (_input && !autoOn)
	{
		_controlState = Manual;
	}
}

MotionController::~MotionController()
{
	_running = false;
	
	if (isRunning())
	{
		wait();
	}
	
	if (_input)
	{
		delete _input;
	}

	//cleanup PID's
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		delete _robots[i];
		_robots[i] = 0;
	}
}

void MotionController::playPause()
{
	if (_runState == Running)
	{
		_runState = Stopped;
		printf("Stopped\n");
	}
	else
	{
		_runState = Running;
		printf("Running\n");
	}
}

void MotionController::autoMan()
{
	if (_controlState == Auto)
	{
		_controlState = Manual;
		printf("Manual\n");
	}
	else
	{
		_controlState = Auto;
		printf("Auto\n");
	}
}

void MotionController::visionDataHandler(const VisionData* data)
{
	_visionData = *data;
	_newVision = true;
}

void MotionController::motionCmdHandler(const MotionCmd* data)
{
	if (data)
	{
		_commands = *data;
	}
	//if motion data timeout, disable it
	else
	{
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			_commands.robots[i].valid = false;
		}
	}
}

void MotionController::refHandler(const Packet::Ref* data)
{
	_ref = *data;
}

void MotionController::run()
{
	// Packet sender for log data and comm data
	Packet::PacketSender sender(_team);

	//packet receiver for vision and commands
	Packet::PacketReceiver packRecv(_team);
	packRecv.addType(this, &MotionController::visionDataHandler);
	packRecv.addType(this, &MotionController::motionCmdHandler, 100);
	packRecv.addType(this, &MotionController::refHandler);
	
	printf("Motion Controller Running.\n");
	printf("Mode: ");
	if (_controlState == Auto)
	{
		printf("Auto\n");
	}
	else
	{
		printf("Manual\n");
	}
	
	printf("State: ");
	if (_runState == Running)
	{
		printf("Running\n");
	}
	else if (_runState == Stopped)
	{
		printf("Stopped\n");
	}
	
	while (_running)
	{
		//set all robots to false
		//robots must be turned on each loop interval
		_commData.invalidate();

		//get new packets and call the handlers
		//this processes all new packets first
		
		//packet receiver SHOULD NOT block
		packRecv.receive();
		
		if (_controlState == Auto && _running)
		{
			//clear any previous zones
			//zones are not stateful
			for (unsigned int i=0 ; i<5 ; ++i)
			{
				Robot* r = _robots[i];
				if (r)
				{
					r->pathPlanner()->clearNoZones();
					r->pathPlanner()->setAvoidOp(false);
				}
			}
		}
		
		//always process ref data in auto because the ball may move
		if (_controlState == Auto && _newVision)
		{
			procRef();
		}

		if (_runState == Running)
		{
			if (_controlState == Auto && _newVision)
			{
				for (unsigned int i=0 ; i<5 ; ++i)
				{
					_robots[i]->proc(_commands.robots[i]);
				}
				
				//handle reverse channel
				if (_commands.baton >= 0 && _commands.baton <= 4)
				{
					_commData.mode = CommData::Fixed;
					_commData.robot = _commands.baton;
				}
				else
				{
					_commData.mode = CommData::ScanRobots;
				}
			}
			//only use controller if there is one (_input != 0)
			else if (_controlState == Manual && _input)
			{
				//clear PID loops, sum should go away
				for (unsigned int i=0 ; i<5 ; ++i)
				{
					_robots[i]->clearPid();
				}
				
				QThread::msleep(10); //process input @ 90Hz, instead of blocking input wait
				
				//create robot data from input device
				_commData.robots[_input->currentRobot()] = _input->genRobotData();
				_commData.timestamp = _visionData.timestamp;
				sender.send(_commData);
			}
			
			if (_newVision && _controlState == Auto)
			{				
				//set the timestamp to their vision parent
				_logData.timestamp = _commData.timestamp = _visionData.timestamp;

				sender.send(_commData);
				sender.send(_logData);
				
				//reset the vision flag
				_newVision = false;
			}
		}
		else
		{
			for (unsigned int i=0 ; i<5 ; ++i)
			{
				_robots[i]->clearPid();
			}
		}
	}
}

void MotionController::procRef()
{
	//halt = no motion
	if (_ref.state == Ref::Halt)
	{
		_runState = Stopped;
	}
	//everything else allows motion but with certain limitations
	else
	{
		_runState = Running;
		bool useNoZone = false;
		Geometry::Circle2d noZone;
		bool avoidOp = false;
		
		if ((_ref.state == Ref::Setup || _ref.state == Ref::OppStart) 
				&& _ref.start == Ref::Kickoff && !_ref.ourStart)
		{
			//stay off opp side of field
			avoidOp = true;
		}
		
		//if we are setup and our start || if running, clear all zones
		if (_ref.state == Ref::Setup && !_ref.ourStart && !_ref.start == Ref::Penalty)
		{
			if (_visionData.ball.valid)
			{
				//stay away from the ball
				noZone = Circle2d(_visionData.ball.pos, BALL_AVOID);
				useNoZone = true;
			}
		}
		
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			Robot* r = _robots[i];
			if (r)
			{
				//r->pathPlanner()->setAvoidOp(avoidOp);
				
				if (useNoZone)
				{
					r->pathPlanner()->addNoZone(noZone);
				}
			}
			_robots[i]->clearPid();
		}
	}	
}
