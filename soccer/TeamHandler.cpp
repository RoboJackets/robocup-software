#include "TeamHandler.hpp"

#include <Network/Network.hpp>
#include <Network/PacketReceiver.hpp>

#include <Vision.hpp>
#include <Constants.hpp>

TeamHandler::TeamHandler(Team t) :
	QThread(), _running(true),
	_team(t)
{
	//default yellow
	Geometry::Point2d trans(0, Constants::Field::Length/2.0f);
	_teamAngle = 90;

	if (_team == Blue)
	{
		_teamAngle = -90;
		trans = Geometry::Point2d(0, Constants::Field::Length/2.0f);
	}

	_teamTrans = Geometry::TransformMatrix::translate(trans);
	_teamTrans *= Geometry::TransformMatrix::rotate(_teamAngle);
	
	//initially no camera does the triggering
	_camTrigId = -1;
	_trigger = false;
}

TeamHandler::~TeamHandler()
{
	_running = false;
	wait();
}

void TeamHandler::run()
{
	Network::PacketReceiver receiver;
	receiver.addType(Network::Address, Network::Vision, this, &TeamHandler::visionHandler);
	//receiver.addType(Network::Address, Network::addTeamOffset(Network::RadioRx), this, &TeamHandler::radioHandler);

	while (_running)
	{
		Packet::Vision visIn;
		
		//receive all packets
		//call packet handlers
		receiver.receive();

		if (_trigger)
		{
			_modulesMutex.lock();
			
			Q_FOREACH(Module* m, _modules)
			{
				m->run();
			}
			
			_modulesMutex.unlock();
			
			//clear system state info
			_state = SystemState();
			
			_trigger = false;
		}
	}
}

void TeamHandler::addModule(Module* module)
{
	_modulesMutex.lock();
	
	module->setSystemState(&_state);
	_modules.push_back(module);
	
	_modulesMutex.unlock();
}

void TeamHandler::visionHandler(const Packet::Vision* packet)
{
	//add the vision packet to the log frame
	_state.rawVision.push_back(*packet);
	
	//process vision frame into system state
	//turn into team space coordinates and place into aggregate vision
	
	Q_FOREACH (const Packet::Vision::Robot& raw , packet->blue)
	{
		Packet::Vision::Robot r;
		
		r.shell = raw.shell;
		r.pos = _teamTrans * raw.pos;
		r.angle = _teamAngle + raw.angle;
		TeamHandler::trim(r.angle);
		
		if (_team == Blue)
		{
			_state.allSelf.push_back(r);
		}
		else
		{
			_state.allOpp.push_back(r);
		}
	}

	Q_FOREACH (const Packet::Vision::Robot& raw , packet->yellow)
	{
		Packet::Vision::Robot r;
		
		r.shell = raw.shell;
		r.pos = _teamTrans * raw.pos;
		r.angle = _teamAngle + raw.angle;
		TeamHandler::trim(r.angle);
		
		if (_team == Yellow)
		{
			_state.allSelf.push_back(r);
		}
		else
		{
			_state.allOpp.push_back(r);
		}
	}

	Q_FOREACH (const Packet::Vision::Ball& raw , packet->balls)
	{
		Packet::Vision::Ball b;
		b.pos = _teamTrans * raw.pos;
		_state.allBalls.push_back(b);
	}
	
	//trigger system
	if (_camTrigId == packet->camera)
	{
		//timestamp of vision is the timestamp of the camera image
		//TODO change to timestamp of writing the log file?
		_state.timestamp = packet->timestamp;
		_trigger = true;
	}
	
	if (_camTrigId == -1)
	{
		_camTrigId = packet->camera;
		printf("Set trigger camera to: %d\n", _camTrigId);
	}
}

#if 0
void TeamHandler::radioHandler(const Packet::RadioRx* packet)
{
	
}
#endif

void TeamHandler::trim(float& angle)
{
	if (angle > 180)
	{
		angle -= 360.0f;
	}
	else if (angle < -180)
	{
		angle += 360.0f;
	}
}
