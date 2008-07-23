#include "Skills.hpp"

using namespace Packet;
using namespace Geometry;

Skills::Skills(Team t) :
	_team(t), _running(true)
{
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_robots[i] = new Robot(i, _vision, _status.robots[i]);
	}
	
	//no robot has baton first
	_baton = 0;
}

Skills::~Skills()
{
	_running = false;
	
	if (isRunning())
	{
		wait();
	}

	for (unsigned int i=0 ; i<5 ; ++i)
	{
		delete _robots[i];
	}
}

void Skills::visionHandler(const Packet::VisionData* data)
{
	_newVisionData = true;
	
	Geometry::Point2d oldBall = _vision.ball.pos;
	
	_vision = *data;
	
	//if new ball data is invalid, then use old position
	if (!_vision.ball.valid)
	{
		_vision.ball.pos = oldBall;
		_vision.ball.vel = Geometry::Point2d(0,0);
	}
}

void Skills::skillCmdHandler(const Packet::SkillCmd* data)
{
	if (data)
	{
		_skillCmd = *data;
	}
	else
	{
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			_skillCmd.robots[i].valid = false;
		}
	}
}

void Skills::statusHandler(const Packet::RobotStatus* data)
{
	if (data)
	{
		_status = *data;
	}
	else
	{
		_status = Packet::RobotStatus();
	}
}

void Skills::run()
{
	PacketReceiver receiver(_team);
	receiver.addType(this, &Skills::visionHandler);
	receiver.addType(this, &Skills::skillCmdHandler, 100);
	receiver.addType(this, &Skills::statusHandler, 100);
	
	PacketSender sender(_team);
	
	//outgoing motion and status
	MotionCmd motion;
	
	printf("Starting Skills for: %s\n", teamToA(_team));
	
	while(_running)
	{
		receiver.receive();
				
		//control the robots
		if (_newVisionData)
		{
            SkillStatus status;
			
			//reset vision data flag
			_newVisionData = false;

			//proc robot skill
			for (unsigned int i=0 ; i<5 ; ++i)
			{
				motion.robots[i].valid = false;
				
				if (_vision.self[i].valid)
				{
					motion.robots[i] = _robots[i]->proc(_skillCmd.robots[i]);
				}
				else
				{
					_robots[i]->releaseBaton();
				}
				
				status.robots[i].status = _robots[i]->skillStatus();
                status.robots[i].sequence = _skillCmd.robots[i].sequence;
			}
			
			//process the baton requests
			procBaton();
			
			motion.timestamp = _vision.timestamp;
			
			//set baton info in motion
			if (_baton)
			{
				motion.baton = _baton->id();
			}
			else
			{
				motion.baton = -1;
			}
			
			sender.send(motion);
			sender.send(status);
		}
	}
}

void Skills::procBaton()
{
	//always reset the baton
	_baton = 0;
	
	//for now give baton to first robot requesting it
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		if (_robots[i]->needBaton())
		{
			_baton = _robots[i];
		}
	}
}		
