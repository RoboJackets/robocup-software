#include "Processor.hpp"
#include "Processor.moc"

#include <QMutexLocker>

#include <Network/Network.hpp>
#include <Network/Sender.hpp>
#include <Network/PacketReceiver.hpp>

#include <Vision.hpp>
#include <Constants.hpp>

Processor::Processor(Team t) :
	_running(true), _team(t), _inputHandler(this)
{
	//default yellow
	Geometry::Point2d trans(0, Constants::Field::Length / 2.0f);
	_teamAngle = 90;

	if (_team == Blue)
	{
		_teamAngle = -90;
		trans = Geometry::Point2d(0, Constants::Field::Length / 2.0f);
	}

	_teamTrans = Geometry::TransformMatrix::translate(trans);
	_teamTrans *= Geometry::TransformMatrix::rotate(_teamAngle);

	//initially no camera does the triggering
	_triggerId = -1;
	_trigger = false;
	
	//record team in state variable
	if (_team == Blue)
	{
		_state.isBlue = true;
	}
	else
	{
		_state.isBlue = false;
	}
	
	_inputHandler.setObjectName("input");
	_inputHandler.start();
	
	QMetaObject::connectSlotsByName(this);
}

Processor::~Processor()
{
	_running = false;
	wait();
}

void Processor::run()
{
	//setup receiver of packets for vision and radio
	Network::PacketReceiver receiver;
	receiver.addType(Network::Address, Network::Vision, this,
	        &Processor::visionHandler);
	receiver.addType(Network::Address, Network::addTeamOffset(_team, Network::RadioRx), 
			this, &Processor::radioHandler);
	
	//sender of outgoing radio control data
	Network::Sender sender(Network::Address, Network::addTeamOffset(_team, Network::RadioTx));
	
	while (_running)
	{
		//needs to be non-blocking...is it?
		receiver.receive();
		
		if (true)
		{
			//auto
		}
		else
		{
			//manual
		}
		
		if (_trigger)
		{
			_modulesMutex.lock();
			
			Q_FOREACH(Module* m, _modules) 
			{
				m->run();
			}

			_modulesMutex.unlock();
			
			sender.send(_state.radioCmd);
			
			//clear system state info
			//TODO should I clear system state?
			//_state = SystemState();
			
			//wait for new trigger frame
			_trigger = false;
		}
	}
}
void Processor::addModule(Module* module)
{
	QMutexLocker ml(&_modulesMutex);

	module->setSystemState(&_state);
	_modules.push_back(module);
}

void Processor::visionHandler(const Packet::Vision* packet)
{
	//detect trigger camera, if not already set
	if (_triggerId < 0 && packet->sync)
	{
		if (_state.rawVision.size() > 0 && _state.rawVision[0].camera
		        == packet->camera)
		{
			//TODO investigate...
			uint64_t half = _state.rawVision[0].timestamp;
			half += (packet->timestamp - _state.rawVision[0].timestamp) / 2;
			
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

void Processor::radioHandler(const Packet::RadioRx* packet)
{
	//log received radio data time
}

void Processor::toTeamSpace(Packet::Vision& vision)
{
	//FIXME FIXME we should put this info into self/opp??
	for (unsigned int i = 0; i < vision.blue.size(); ++i)
	{
		Packet::Vision::Robot& r = vision.blue[i];
		r.pos = _teamTrans * r.pos;
		r.angle = _teamAngle + r.angle;
		Processor::trim(r.angle);
	}

	for (unsigned int i = 0; i < vision.yellow.size(); ++i)
	{
		Packet::Vision::Robot& r = vision.yellow[i];
		r.pos = _teamTrans * r.pos;
		r.angle = _teamAngle + r.angle;
		Processor::trim(r.angle);
	}
	
	for (unsigned int i = 0; i < vision.balls.size(); ++i)
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

/// slots ///
void Processor::on_input_playPauseButton()
{
	switch (_state.runState)
	{
		case SystemState::Stopped:
			_state.runState = SystemState::Running;
			printf("Running\n");
			break;
		case SystemState::Running:
		default:
			_state.runState = SystemState::Stopped;
			printf("Stopped\n");
			break;
	}
}

void Processor::on_input_manualAutoButton()
{
	switch (_state.controlState)
	{
		case SystemState::Manual:
			_state.controlState = SystemState::Auto;
			break;
		case SystemState::Auto:
		default:
			_state.controlState = SystemState::Manual;
			break;
	}
}

void Processor::on_input_changeRobot(int rid)
{
	_state.rid = rid;
}
