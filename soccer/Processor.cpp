#include "Processor.hpp"

#include <Network/Network.hpp>
#include <Network/PacketReceiver.hpp>

#include <Vision.hpp>
#include <Constants.hpp>

Processor::Processor(Team t) :
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
	_triggerId = -1;
	_trigger = false;
}

Processor::~Processor()
{
	_running = false;
	wait();
}

void Processor::run()
{
	Network::PacketReceiver receiver;
	receiver.addType(Network::Address, Network::Vision, this, &Processor::visionHandler);
	//receiver.addType(Network::Address, Network::addTeamOffset(Network::RadioRx), this, &TeamHandler::radioHandler);

	while (_running)
	{
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

void Processor::addModule(Module* module)
{
	_modulesMutex.lock();
	
	module->setSystemState(&_state);
	_modules.push_back(module);
	
	_modulesMutex.unlock();
}

void Processor::visionHandler(const Packet::Vision* packet)
{
	//detect trigger camera, if not already set
	if (_triggerId < 0 && packet->sync)
	{	
		if (_state.rawVision.size() > 0 && _state.rawVision[0].camera == packet->camera)
		{
			uint64_t half = _state.rawVision[0].timestamp;
			half += (packet->timestamp - _state.rawVision[0].timestamp)/2;
			
			//eval all packets, any packet less than half becomes the new tigger
			//this gives us the last packet up to half as the trigger
			Q_FOREACH (const Packet::Vision& raw , _state.rawVision)
			{
				if (raw.timestamp < half)
				{
					_triggerId = raw.camera;
				}
			}
			
			//we have set the trigger camera
			printf("Set trigger camera: %d\n", _triggerId);
			_state.rawVision.clear();
			return;
		}
		
		//store received packets in the log frame
		_state.rawVision.push_back(*packet);
		
		return;
	}
	
	//add the packet to the list of vision to process
	//this also includes sync messages, which will need to be ignored
	_state.rawVision.push_back(*packet);
	
	//convert last frame to teamspace
	toTeamSpace(_state.rawVision[_state.rawVision.size() - 1]);
	
	if (packet->camera == _triggerId)
	{
		if (packet->sync)
		{
			//printf("radio: %d\n", packet->camera);
			//TODO tx radio data
		}
		else
		{
			//set syncronous time to packet timestamp
			_state.timestamp = packet->timestamp;
			
			//start s.proc
			_trigger = true;
		}
	}
}

#if 0
void TeamHandler::radioHandler(const Packet::RadioRx* packet)
{
	//log received radio data time
}
#endif

void Processor::toTeamSpace(Packet::Vision& vision)
{
	//FIXME FIXME we should put this info into self/opp?? 
	
	for (unsigned int i=0 ; i< vision.blue.size() ; ++i)
	{
		Packet::Vision::Robot& r = vision.blue[i];
		r.pos = _teamTrans * r.pos;
		r.angle = _teamAngle + r.angle;
		Processor::trim(r.angle);
	}

	for (unsigned int i=0 ; i<vision.yellow.size() ; ++i)
	{
		Packet::Vision::Robot& r = vision.yellow[i];
		r.pos = _teamTrans * r.pos;
		r.angle = _teamAngle + r.angle;
		Processor::trim(r.angle);
	}

	for (unsigned int i=0 ; i< vision.balls.size() ; ++i)
	{
		Packet::Vision::Ball& b = vision.balls[i];
		b.pos = _teamTrans * b.pos;
	}
}

void Processor::trim(float& angle)
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
