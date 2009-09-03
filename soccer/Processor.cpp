// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "Processor.hpp"
#include "Processor.moc"

#include <QMutexLocker>

#include <Network/Network.hpp>
#include <Network/Sender.hpp>
#include <Network/PacketReceiver.hpp>

#include <Point.hpp>
#include <Vision.hpp>
#include <Constants.hpp>
#include <Utils.hpp>

#include <modeling/WorldModel.hpp>
#include <log/LogModule.hpp>
#include <gameplay/GameplayModule.hpp>
#include <boost/foreach.hpp>

using namespace std;

Processor::Processor(Team t, QString filename) :
	_running(true),
	_team(t),
	_sender(Network::Address, Network::addTeamOffset(_team, Network::RadioTx)),
	_config(filename),
	_slowThread(this)
{
	Geometry2d::Point trans;
	if (_team == Blue)
	{
		_teamAngle = -90;
		trans = Geometry2d::Point(0, Constants::Field::Length / 2.0f);
	} else {
		// Assume yellow
		_teamAngle = 90;
		trans = Geometry2d::Point(0, Constants::Field::Length / 2.0f);
	}

	//transformations from world to team space
	_teamTrans = Geometry2d::TransformMatrix::translate(trans);
	_teamTrans *= Geometry2d::TransformMatrix::rotate(_teamAngle);

	_flipField = false;

	//set the team
	_state.team = _team;

	//initially no camera does the triggering
	_triggerId = -1;
	_trigger = false;

	//runs independently of main loop
	_inputHandler = new InputHandler(this);
	_inputHandler->setObjectName("input");
	_inputHandler->start();

	//default to auto, running when no input device
	if (!_inputHandler->enabled())
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
	_modelingModule = new Modeling::WorldModel(_config.worldModel);
	_motionModule = new Motion::MotionModule(_config.motionModule);
	_refereeModule = new RefereeModule();
	_gameplayModule = new Gameplay::GameplayModule();
	_logModule = new Log::LogModule();

	_modules.append(_modelingModule);
	_modules.append(_refereeModule);
	_modules.append(_gameplayModule);
	_modules.append(_motionModule);
	_modules.append(_logModule);

	BOOST_FOREACH(Module *module, _modules)
	{
		module->state(&_state);
	}
}

void Processor::start()
{
	QThread::start();
	//_slowThread.start();
}

Processor::~Processor()
{
	_running = false;
	wait();

	if (_inputHandler)
	{
		delete _inputHandler;
	}

	BOOST_FOREACH(Module *module, _modules)
	{
		delete module;
	}
}

void Processor::setLogFile(Log::LogFile* lf)
{
	_logModule->setLogFile(lf);
}

void Processor::SlowThread::run()
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
			_refereeModule, &RefereeModule::packet);

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
				_refereeModule->run();

				// Clear radio commands and get shell numbers from world model
				for (int r = 0; r < Constants::Robots_Per_Team; ++r)
				{
					_state.self[r].radioTx = Packet::RadioTx::Robot();
					_state.self[r].radioTx.board_id = _state.self[r].shell;
				}

				if (_state.manualID >= 0)
				{
					_inputHandler->genRobotData(_state.self[_state.manualID].radioTx);
				}

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
					// Clear radio commands
					for (int r = 0; r < 5; ++r)
					{
						_state.self[r].radioTx = Packet::RadioTx::Robot();
					}

					if (_modelingModule)
					{
						_modelingModule->run();
					}
					
					for (int r = 0; r < 5; ++r)
					{
						if (_state.self[r].valid)
						{
							ConfigFile::Robot* rcfg = _config.robot(_state.self[r].shell);
							
							if (rcfg)
							{
								//printf("%f\n", rcfg->motion.deg0.velocity);
								_state.self[r].config = *rcfg;
							}
						}
					}
					
					if (_refereeModule)
					{
						_refereeModule->run();
					}

					if (_gameplayModule)
					{
						_gameplayModule->run();
					}

					if (_motionModule)
					{
						_motionModule->run();
					}

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
			_refereeModule->run();

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

	bool halt;
	if (_state.controlState == SystemState::Manual)
	{
		// Manual
		halt = (_state.runState != SystemState::Running);
	} else {
		// Auto
		halt = _state.gameState.halt();
	}

	if (halt)
	{
		// Force all motor speeds to zero
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

				//default trigger to this camera
				_triggerId = packet->camera;

				//eval all packets, any packet less than half becomes the new tigger
				//this gives us the last packet up to half as the trigger
				BOOST_FOREACH (const Packet::Vision& raw , _state.rawVision)
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

	if (!packet->sync)
	{
		//set syncronous time to packet timestamp
		_state.timestamp = packet->timestamp;

		//convert last frame to teamspace
		toTeamSpace(_state.rawVision[_state.rawVision.size() - 1]);
	}
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

		if (_flipField)
		{
			r.pos *= -1;
			r.angle = Utils::fixAngleDegrees(r.angle + 180);
		}

		r.pos = _teamTrans * r.pos;
		r.angle = Utils::fixAngleDegrees(_teamAngle + r.angle);
	}

	for (unsigned int i = 0; i < vision.yellow.size(); ++i)
	{
		Packet::Vision::Robot& r = vision.yellow[i];

		if (_flipField)
		{
			r.pos *= -1;
			r.angle = Utils::fixAngleDegrees(r.angle + 180);
		}

		r.pos = _teamTrans * r.pos;
		r.angle = Utils::fixAngleDegrees(_teamAngle + r.angle);
	}

	for (unsigned int i = 0; i < vision.balls.size(); ++i)
	{
		Packet::Vision::Ball& b = vision.balls[i];

		if (_flipField)
		{
			b.pos *= -1;
		}

		b.pos = _teamTrans * b.pos;
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

void Processor::on_input_selectRobot(int rid)
{
	_state.manualID = rid;
	printf ("Controlling robot: %d\n", _state.manualID);
}

void Processor::flip_field(bool flip)
{
	_flipField = flip;
}
