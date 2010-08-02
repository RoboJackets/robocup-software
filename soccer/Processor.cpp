// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "Processor.hpp"

#include <QMutexLocker>

#include <poll.h>
#include <multicast.hpp>
#include <Constants.hpp>
#include <Network.hpp>
#include <Utils.hpp>
#include <Joystick.hpp>
#include <LogUtils.hpp>

#include <modeling/WorldModel.hpp>
#include <gameplay/GameplayModule.hpp>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>

using namespace std;
using namespace boost;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

Processor::Processor(QString filename, bool sim, int radio) :
	_config(new ConfigFile(filename))
{
	_running = true;
	_syncToVision = false;
	_reverseId = 0;
	_framePeriod = 1000000 / 60;
	_manualID = -1;
	_defendPlusX = false;
	_state.logFrame = &logFrame;
	_externalReferee = true;
	_framerate = 0;
	firstLogTime = 0;

	_simulation = sim;
	_joystick = new Joystick();

	// Initialize team-space transformation
	defendPlusX(_defendPlusX);
	
	QMetaObject::connectSlotsByName(this);

	try
	{
		_config->load();
	}
	catch (std::runtime_error& re)
	{
		printf("Config Load Error: %s\n", re.what());
	}

	// Create vision socket
	if (_simulation)
	{
		// The simulator doesn't multicast its vision.  Instead, it sends to two different ports.
		// Try to bind to the first one and, if that fails, use the second one.
		if (!_visionSocket.bind(SimVisionPort))
		{
			if (!_visionSocket.bind(SimVisionPort + 1))
			{
				throw runtime_error("Can't bind to either simulated vision port");
			}
		}
	} else {
		// Receive multicast packets from shared vision.
		if (!_visionSocket.bind(SharedVisionPort, QUdpSocket::ShareAddress))
		{
			throw runtime_error("Can't bind to shared vision port");
		}
		
		multicast_add(_visionSocket, SharedVisionAddress);
	}

	// Create referee socket
	if (!_refereeSocket.bind(RefereePort, QUdpSocket::ShareAddress))
	{
		throw runtime_error("Can't bind to referee port");
	}
	
	multicast_add(_refereeSocket, RefereeAddress);

	// Create radio socket
	if (radio < 0)
	{
		// No channel specified.
		// Pick the first available one.
		if (_radioSocket.bind(RadioRxPort))
		{
			_radio = 0;
		} else {
			if (_radioSocket.bind(RadioRxPort + 1))
			{
				_radio = 1;
			} else {
				throw runtime_error("Can't bind to either radio port");
			}
		}
	} else {
		// Bind only to the port for the specified channel.
		if (!_radioSocket.bind(RadioRxPort + radio))
		{
			throw runtime_error("Can't bind to specified radio port");
		}
		_radio = radio;
	}
	
	//setup the modules
	_modelingModule = make_shared<Modeling::WorldModel>(&_state, _config->worldModel);
	_stateIDModule = make_shared<StateIdentification::StateIDModule>(&_state);
	_pointControlModule = make_shared<Motion::PointController>(&_state, _config->motionModule);
	_wheelControlModule = make_shared<Motion::WheelController>(&_state, _config->motionModule);
//	_motionModule = make_shared<Motion::MotionModule>(&_state, _config->motionModule); // FIXME: remove
	_refereeModule = make_shared<RefereeModule>(&_state);
	_gameplayModule = make_shared<Gameplay::GameplayModule>(&_state, _config->motionModule);
}

Processor::~Processor()
{
	stop();
	
	delete _joystick;
	
	//DEBUG - This is unnecessary, but lets us determine which one breaks.
	_modelingModule.reset();
	_stateIDModule.reset();
	_pointControlModule.reset();
	_wheelControlModule.reset();
//	_motionModule.reset(); //FIXME: remove
	_refereeModule.reset();
	_gameplayModule.reset();
}

void Processor::stop()
{
	if (_running)
	{
		_running = false;
		wait();
	}
}

void Processor::manualID(int value)
{
	QMutexLocker locker(&_loopMutex);
	_manualID = value;
}

void Processor::blueTeam(bool value)
{
	// This is called from the GUI thread
	QMutexLocker locker(&_loopMutex);
	
	_blueTeam = value;
}

void Processor::internalRefCommand(char ch)
{
	QMutexLocker locker(&_loopMutex);
	
	// Change scores
	switch (ch)
	{
		case RefereeCommands::GoalBlue:
			if (blueTeam())
			{
				++state()->gameState.ourScore;
			} else {
				++state()->gameState.theirScore;
			}
			break;
		
		case RefereeCommands::SubtractGoalBlue:
			if (blueTeam())
			{
				if (state()->gameState.ourScore)
				{
					--state()->gameState.ourScore;
				}
			} else {
				if (state()->gameState.theirScore)
				{
					--state()->gameState.theirScore;
				}
			}
			break;
		
		case RefereeCommands::GoalYellow:
			if (blueTeam())
			{
				++state()->gameState.theirScore;
			} else {
				++state()->gameState.ourScore;
			}
			break;
		
		case RefereeCommands::SubtractGoalYellow:
			if (blueTeam())
			{
				if (state()->gameState.theirScore)
				{
					--state()->gameState.theirScore;
				}
			} else {
				if (state()->gameState.ourScore)
				{
					--state()->gameState.ourScore;
				}
			}
			break;
	}
	
	// Send the command to the referee handler
	_refereeModule->command(ch);
}

void Processor::addMotors(RadioTx::Robot* robot)
{
	for (size_t m = 0; m < 4; ++m)
	{
		robot->add_motors(0);
	}
}

bool Processor::autonomous()
{
	QMutexLocker lock(&_loopMutex);
	return _joystick->autonomous();
}

bool Processor::joystickValid()
{
	QMutexLocker lock(&_loopMutex);
	return _joystick->valid();
}

void Processor::run()
{
	Status curStatus;
	
	while (_running)
	{
		uint64_t startTime = Utils::timestamp();
		int delta_us = startTime - curStatus.lastLoopTime;
		_framerate = 1000000.0 / delta_us;
		curStatus.lastLoopTime = startTime;
		_state.timestamp = startTime;
		
		if (!firstLogTime)
		{
			firstLogTime = startTime;
		}
		
		////////////////
		// Reset
		
		// Reset the log frame
		logFrame.Clear();
		logFrame.set_start_time(startTime);
		
		// Clear radio commands
		for (size_t r = 0; r < Constants::Robots_Per_Team; ++r)
		{
			_state.self[r].radioTx = 0;
		}

		////////////////
		// Inputs
		
		// Read vision packets
		vector<const SSL_DetectionFrame *> rawVision;
		int timeout = _framePeriod / 1000;
		while (_syncToVision || _visionSocket.hasPendingDatagrams())
		{
			if (_syncToVision)
			{
				struct pollfd pfd;
				pfd.fd = _visionSocket.socketDescriptor();
				pfd.events = POLLIN;
				if (poll(&pfd, 1, timeout) == 0)
				{
					// Timeout
					break;
				}
				timeout = 0;
			}
			
			string buf;
			unsigned int n = _visionSocket.pendingDatagramSize();
			buf.resize(n);
			_visionSocket.readDatagram(&buf[0], n);
			
			SSL_WrapperPacket *packet = logFrame.add_raw_vision();
			if (!packet->ParseFromString(buf))
			{
				printf("Bad vision packet of %d bytes\n", n);
				continue;
			}
			
			curStatus.lastVisionTime = Utils::timestamp();
			if (packet->has_detection())
			{
				rawVision.push_back(&packet->detection());
			}
		}
		
		// Read referee packets
		while (_refereeSocket.hasPendingDatagrams())
		{
			unsigned int n = _refereeSocket.pendingDatagramSize();
			string str(6, 0);
			_refereeSocket.readDatagram(&str[0], str.size());
			
			// Check the size after receiving to discard bad packets
			if (n != str.size())
			{
				printf("Bad referee packet of %d bytes\n", n);
				continue;
			}
			
			// Log the referee packet, but only use it if external referee is enabled
			curStatus.lastRefereeTime = Utils::timestamp();
			logFrame.add_raw_referee(str);
			
			if (_externalReferee)
			{
				_refereeModule->packet(str);
			}
		}
		
		// Read radio RX packets
		while (_radioSocket.hasPendingDatagrams())
		{
			unsigned int n = _radioSocket.pendingDatagramSize();
			string buf;
			buf.resize(n);
			_radioSocket.readDatagram(&buf[0], n);
			
			RadioRx *rx = logFrame.add_radio_rx();
			if (!rx->ParseFromString(buf))
			{
				printf("Bad radio packet of %d bytes\n", n);
				continue;
			}
			
			curStatus.lastRadioRxTime = Utils::timestamp();
			
			// Store this packet in the appropriate robot
			for (size_t i = 0 ; i < Constants::Robots_Per_Team; ++i)
			{
				if (_state.self[i].shell == rx->board_id())
				{
					// We have to copy because the RX packet will survive past this frame
					// but LogFrame will not (the RadioRx in LogFrame will be reused).
					_state.self[i].radioRx.CopyFrom(*rx);
					break;
				}
			}
		}
		
		_loopMutex.lock();
		
		_joystick->update();
		
		_modelingModule->run(_blueTeam, rawVision);
		
		// Convert modeling output to team space
		_state.ball.pos = _worldToTeam * (_state.ball.pos / 1000.0f);
		_state.ball.vel = _worldToTeam.transformDirection(_state.ball.vel / 1000.0f);
		_state.ball.accel = _worldToTeam.transformDirection(_state.ball.accel / 1000.0f);
		
		BOOST_FOREACH(SystemState::Robot &robot, _state.self)
		{
			robot.pos = _worldToTeam * (robot.pos / 1000.0f);
			robot.vel = _worldToTeam.transformDirection(robot.vel / 1000.0f);
			robot.angle = Utils::fixAngleDegrees(_teamAngle + robot.angle);
		}
		
		BOOST_FOREACH(SystemState::Robot &robot, _state.opp)
		{
			robot.pos = _worldToTeam * (robot.pos / 1000.0f);
			robot.vel = _worldToTeam.transformDirection(robot.vel / 1000.0f);
			robot.angle = Utils::fixAngleDegrees(_teamAngle + robot.angle);
		}
		
		// Add RadioTx commands for visible robots
		for (size_t r = 0; r < Constants::Robots_Per_Team; ++r)
		{
			if (_state.self[r].valid)
			{
				RadioTx::Robot *tx = logFrame.mutable_radio_tx()->add_robots();
				_state.self[r].radioTx = tx;
				tx->set_board_id(_state.self[r].shell);
				addMotors(tx);
			}
		}
		
		if (_refereeModule)
		{
			_refereeModule->run();
		}
		
		for (size_t r = 0; r < Constants::Robots_Per_Team; ++r)
		{
			if (_state.self[r].valid)
			{
				ConfigFile::shared_robot rcfg = _config->robot(_state.self[r].shell);
				
				if (rcfg)
				{
					_state.self[r].config = *rcfg;

					// set the config information
					switch (rcfg->rev) {
					case ConfigFile::rev2008:
						_state.self[r].rev =  SystemState::Robot::rev2008;
						break;
					case ConfigFile::rev2010:
						_state.self[r].rev =  SystemState::Robot::rev2010;
					}
				}
			}
		}
		
		if (_stateIDModule)
		{
			_stateIDModule->run();
		}

		if (_gameplayModule)
		{
			_gameplayModule->run();
		}

		// FIXME: remove
//		if (_motionModule)
//		{
//			_motionModule->run();
//		}

		if (_pointControlModule)
		{
			_pointControlModule->run();
		}

		if (_wheelControlModule)
		{
			_wheelControlModule->run();
		}
		
		////////////////
		// Store logging information
		
		logFrame.set_manual_id(_manualID);
		logFrame.set_blue_team(_blueTeam);
		logFrame.set_defend_plus_x(_defendPlusX);
		logFrame.set_play(_gameplayModule->playName().toStdString());
		
		// Debug layers
		const QStringList &layers = _state.debugLayers();
		BOOST_FOREACH(const QString &str, layers)
		{
			logFrame.add_debug_layers(str.toStdString());
		}
		
		// Filtered pose
		BOOST_FOREACH(const SystemState::Robot &r, _state.self)
		{
			if (r.valid)
			{
				LogFrame::Robot *log = logFrame.add_self();
				r.pos.set(log->mutable_pos());
				log->set_shell(r.shell);
				log->set_angle(r.angle);
				log->set_has_ball(r.hasBall);
			}
		}
		
		BOOST_FOREACH(const SystemState::Robot &r, _state.opp)
		{
			if (r.valid)
			{
				LogFrame::Robot *log = logFrame.add_opp();
				r.pos.set(log->mutable_pos());
				log->set_shell(r.shell);
				log->set_angle(r.angle);
				log->set_has_ball(r.hasBall);
			}
		}
		
		if (_state.ball.valid)
		{
			LogFrame::Ball *log = logFrame.mutable_ball();
			_state.ball.pos.set(log->mutable_pos());
			_state.ball.vel.set(log->mutable_vel());
		}
		
		////////////////
		// Outputs
		
		// Send motion commands to the robots
		sendRadioData();

		// Write to the log
		logger.addFrame(logFrame);
		
		_loopMutex.unlock();
		
		// Store processing loop status
		_statusMutex.lock();
		_status = curStatus;
		_statusMutex.unlock();
		
		////////////////
		// Timing
		
		uint64_t endTime = Utils::timestamp();
		int lastFrameTime = endTime - startTime;
		if (lastFrameTime < _framePeriod)
		{
			if (!_syncToVision)
			{
				usleep(_framePeriod - lastFrameTime);
			}
		} else {
			printf("Processor took too long: %d us\n", lastFrameTime);
		}
	}
}

void Processor::sendRadioData()
{
	// Cycle through reverse IDs
	logFrame.mutable_radio_tx()->set_reverse_board_id(_state.self[_reverseId].shell);
	_reverseId = (_reverseId + 1) % Constants::Robots_Per_Team;
	
	// Halt overrides normal motion control
	if (_joystick->autonomous() && _state.gameState.halt())
	{
		// Force all motor speeds to zero
		for (int r = 0; r < logFrame.mutable_radio_tx()->robots_size(); ++r)
		{
			RadioTx::Robot *robot = logFrame.mutable_radio_tx()->mutable_robots(r);
			for (int m = 0; m < robot->motors_size(); ++m)
			{
				robot->set_motors(m, 0);
			}
		}
	}

	// Apply joystick input
	bool manualDone = false;
	for (size_t i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		if (_state.self[i].valid)
		{
			RadioTx::Robot *tx = _state.self[i].radioTx;
			if (_manualID >= 0 && _state.self[i].shell == _manualID)
			{
				// Drive this robot manually
				_joystick->drive(tx);
				manualDone = true;
			} else if (!_joystick->autonomous())
			{
				// Stop this robot
				for (int m = 0; m < tx->motors_size(); ++m)
				{
					tx->set_motors(m, 0);
				}
			}
		}
	}
	
	if (_manualID >= 0 && logFrame.radio_tx().robots_size() < Constants::Robots_Per_Team && !manualDone)
	{
		// The manual robot wasn't found by vision/modeling but we have room for it in the packet.
		// This allows us to drive an off-field robot for testing or to drive a robot back onto the field.
		RadioTx::Robot *robot = logFrame.mutable_radio_tx()->add_robots();
		robot->set_board_id(_manualID);
		addMotors(robot);
		_joystick->drive(robot);
	}

	// Send the packet
	std::string out;
	logFrame.radio_tx().SerializeToString(&out);
	_radioSocket.writeDatagram(&out[0], out.size(), LocalAddress, RadioTxPort + _radio);
}

void Processor::defendPlusX(bool value)
{
	_defendPlusX = value;

	if (_defendPlusX)
	{
		_teamAngle = -90;
	} else {
		_teamAngle = 90;
	}

	_worldToTeam = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Constants::Field::Length / 2.0f));
	_worldToTeam *= Geometry2d::TransformMatrix::rotate(_teamAngle);
}
