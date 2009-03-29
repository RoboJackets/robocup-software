// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#include "Processor.hpp"
#include "Processor.moc"

#include <QMutexLocker>

#include <Network/Network.hpp>
#include <Network/Sender.hpp>
#include <Network/PacketReceiver.hpp>

#include <Point2d.hpp>
#include <Vision.hpp>
#include <Constants.hpp>

#include <modeling/WorldModel.hpp>
#include <log/LogModule.hpp>
#include <motion/Controller.hpp>

Processor::Processor(Team t, QString filename) :
	_running(true), _team(t), _inputHandler(this),
	_refereeHandler(&_state),
	_sender(Network::Address, Network::addTeamOffset(_team, Network::RadioTx)),
	_config(filename)
{
	//default yellow
	Geometry::Point2d trans(0, Constants::Field::Length / 2.0f);
	_teamAngle = 90;

	if (_team == Blue)
	{
		_teamAngle = -90;
		trans = Geometry::Point2d(0, Constants::Field::Length / 2.0f);
	}

	//transoformatons from world->teamspace
	_teamTrans = Geometry::TransformMatrix::translate(trans);
	_teamTrans *= Geometry::TransformMatrix::rotate(_teamAngle);

	//initially no camera does the triggering
	_triggerId = -1;
	_trigger = false;

	//runs independently of main loop
	_inputHandler.setObjectName("input");
	_inputHandler.start();

	///setup system state
	//set the team
	_state.team = _team;

	//default to auto, running when no input device
	if (!_inputHandler.enabled())
	{
		printf("No controller: Auto/Running\n");
		_state.controlState = SystemState::Auto;
		_state.runState = SystemState::Running;
	}

	QMetaObject::connectSlotsByName(this);

	try
	{
		_config.load();
	}
	catch (std::runtime_error& re)
	{
		printf("Config Load Error: %s\n", re.what());
	}
	
	//setup the modules
	_motionModule = _modelingModule = _logModule = 0;
	
	_modelingModule = new Modeling::WorldModel(_config.robotFilterConfig());
	_motionModule = new Motion::Controller(_config.robotConfig());
	_logModule = new Log::LogModule();

	//TODO fixme...I dunno..this needs to be done for all the modules
	_logModule->setSystemState(&_state);
	_motionModule->setSystemState(&_state);
	_modelingModule->setSystemState(&_state);
	
	_state.team = _team;
}

Processor::~Processor()
{
	_running = false;
	wait();

	if (_modelingModule)
	{
		delete _modelingModule;
		_modelingModule = 0;
	}

	if (_logModule)
	{
		delete _logModule;
		_logModule = 0;
	}

	if (_motionModule)
	{
		delete _motionModule;
		_motionModule = 0;
	}
}

void Processor::setLogFile(Log::LogFile* lf)
{
	_logModule->setLogFile(lf);
}

void Processor::refereeReceived(const Packet::Referee *packet)
{
}

void Processor::run()
{
	//setup receiver of packets for vision and radio
	Network::PacketReceiver receiver;
	receiver.addType(Network::Address, Network::Vision, this,
			&Processor::visionHandler);
	receiver.addType(Network::Address,
			Network::addTeamOffset(_team, Network::RadioRx),
			this, &Processor::radioHandler);
	receiver.addType(RefereeAddress, RefereePort,
			&_refereeHandler, &RefereeHandler::packet);

	while (_running)
	{
		if (_state.runState == SystemState::Running)
		{
			if (_state.controlState == SystemState::Manual)
			{
				//non blocking information for manual control
				receiver.receive(false);

				//run modeling for testing
				_modelingModule->run();
				_refereeHandler.run();
				
				// Clear radio commands
				for (int r = 0; r < 5; ++r)
				{
					_state.self[r].radioTx = Packet::RadioTx::Robot();
				}
				
				_state.self[_state.rid].radioTx = _inputHandler.genRobotData();
				
				_logModule->run();

				//send out the radio data from manual control
				sendRadioData();

				clearState();

				//constant time wait for radio consistency
				QThread::msleep(35);
			}
			else if (_state.controlState == SystemState::Auto)
			{
				//blocking to act on new packets
				receiver.receive(true);
				
				//if vision told us to act
				if (_trigger)
				{
					_modelingModule->run();
					_refereeHandler.run();
					_motionModule->run();

					//always run logging last
					_logModule->run();
					
					//new state
					clearState();
					
					//wait for new trigger frame
					_trigger = false;
				}
			}
		}
		else
		{
			//blocking to act on new packets
			receiver.receive(false);

			//we should never do anything until processor
			//has established a trigger id... -Roman
			//what if there is no vision??
			
			//run modeling for testing
			_modelingModule->run();
			_refereeHandler.run();
			_logModule->run();
			
			clearState();
			
			//fixed wait
			QThread::msleep(35);
		}
	}
}

void Processor::clearState()
{
	//always clear the raw vision packets
	_state.rawVision.clear();
}

void Processor::sendRadioData()
{
	Packet::RadioTx tx;
	
	for (int i = 0; i < 5; ++i)
	{
		tx.robots[i] = _state.self[i].radioTx;
	}

	bool halt = _refereeHandler.halt() || _state.runState != SystemState::Running;
	if (halt)
	{
		for (int r = 0; r < 5; ++r)
		{
			for (int m = 0; m < 4; ++m)
			{
				tx.robots[r].motors[m] = 0;
			}
		}
	}
	
	_sender.send(tx);
}

void Processor::visionHandler(const Packet::Vision* packet)
{
	//detect trigger camera, if not already set
	if (_triggerId < 0)
	{
		if (packet->sync)
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
				
				//clear state, this will cause only the trigger frame's sync
				//to be stored and system will continue normal operation
				clearState();
			}

			//store the sync packets
			_state.rawVision.push_back(*packet);
		}
		
		return;
	}
	
	//if we have trigger camera, we will either send data
	//or set the trigger flag
	if (packet->camera == _triggerId 
			&& _state.controlState == SystemState::Auto
			&& _state.runState == SystemState::Running)
	{
		//if its a sync packet from trigger camera, then send radio data
		//otherwise set state timestamp, set trigger flag and the system will
		//process modules and send out data
		if (packet->sync)
		{
			//send first
			sendRadioData();
		}
		else
		{
			_trigger = true;
		}
	}
	
	//populate the state
	_state.rawVision.push_back(*packet);

	//set syncronous time to packet timestamp
	_state.timestamp = packet->timestamp;

	//convert last frame to teamspace
	toTeamSpace(_state.rawVision[_state.rawVision.size() - 1]);
}

void Processor::radioHandler(const Packet::RadioRx* packet)
{
	//received radio packets
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_state.self[i].radioRx = packet->robots[i];
	}
}

void Processor::toTeamSpace(Packet::Vision& vision)
{
	//translates raw vision into team space
	//means modeling doesn't need to do it

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
			printf ("Auto mode\n");
			break;
		case SystemState::Auto:
		default:
			_state.controlState = SystemState::Manual;
			printf ("Manual mode\n");
			break;
	}
}

void Processor::on_input_changeRobot(int rid)
{
	if (_state.controlState == SystemState::Manual)
	{
		_state.rid = rid;
		printf ("Controlling robot: %d\n", _state.rid);
	}
}
