// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "Processor.hpp"
#include "radio/SimRadio.hpp"
#include "radio/USBRadio.hpp"

#include <QMutexLocker>

#include <poll.h>
#include <multicast.hpp>
#include <Constants.hpp>
#include <Network.hpp>
#include <Utils.hpp>
#include <Joystick.hpp>
#include <LogUtils.hpp>
#include <BallSensor.hpp>

#include <modeling/WorldModel.hpp>
#include <gameplay/GameplayModule.hpp>
#include <motion/PointControlModule.hpp>
#include <motion/WheelControlModule.hpp>
#include <modeling/WorldModel.hpp>
#include <RefereeModule.hpp>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <git_version.h>

using namespace std;
using namespace boost;
using namespace Packet;
using namespace google::protobuf;

Processor::Processor(Configuration *config, bool sim)
{
	_running = true;
	_syncToVision = false;
	_reverseId = 0;
	_framePeriod = 1000000 / 60;
	_manualID = -1;
	_defendPlusX = false;
	_externalReferee = true;
	_framerate = 0;
	firstLogTime = 0;
	_useOurHalf = true;
	_useOpponentHalf = true;

	_simulation = sim;
	_radio = 0;
	_joystick = new Joystick();
	
	// Initialize team-space transformation
	defendPlusX(_defendPlusX);
	
	QMetaObject::connectSlotsByName(this);

	_modelingModule = make_shared<Modeling::WorldModel>(&_state, config);
	_pointControlModule = make_shared<Motion::PointControlModule>(&_state, config);
	_wheelControlModule = make_shared<Motion::WheelControlModule>(&_state, config);
	_refereeModule = make_shared<RefereeModule>(&_state);
	_gameplayModule = make_shared<Gameplay::GameplayModule>(&_state);
}

Processor::~Processor()
{
	stop();
	
	delete _joystick;
	
	//DEBUG - This is unnecessary, but lets us determine which one breaks.
	_modelingModule.reset();
	_pointControlModule.reset();
	_wheelControlModule.reset();
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
	_refereeModule->blueTeam(value);
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
	_visionSocket = new QUdpSocket;
	_refereeSocket = new QUdpSocket;
	
	// Create vision socket
	if (_simulation)
	{
		// The simulator doesn't multicast its vision.  Instead, it sends to two different ports.
		// Try to bind to the first one and, if that fails, use the second one.
		if (!_visionSocket->bind(SimVisionPort))
		{
			if (!_visionSocket->bind(SimVisionPort + 1))
			{
				throw runtime_error("Can't bind to either simulated vision port");
			}
		}
	} else {
		// Receive multicast packets from shared vision.
		if (!_visionSocket->bind(SharedVisionPort, QUdpSocket::ShareAddress))
		{
			throw runtime_error("Can't bind to shared vision port");
		}
		
		multicast_add(_visionSocket, SharedVisionAddress);
	}

	// Create referee socket
	if (!_refereeSocket->bind(RefereePort, QUdpSocket::ShareAddress))
	{
		throw runtime_error("Can't bind to referee port");
	}
	
	multicast_add(_refereeSocket, RefereeAddress);

	// Create radio socket
	if (_simulation)
	{
		_radio = new SimRadio();
	} else {
		_radio = new USBRadio();
	}
	
	Status curStatus;
	
	bool first = true;
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
		
		// Make a new log frame
		_state.logFrame = make_shared<LogFrame>();
		_state.logFrame->set_start_time(startTime);
		_state.logFrame->set_use_our_half(_useOurHalf);
		_state.logFrame->set_use_opponent_half(_useOpponentHalf);
		_state.logFrame->set_manual_id(_manualID);
		_state.logFrame->set_blue_team(_blueTeam);
		_state.logFrame->set_defend_plus_x(_defendPlusX);
		
		if (first)
		{
			first = false;
			
			LogConfig *logConfig = _state.logFrame->mutable_log_config();
			logConfig->set_generator("soccer");
			logConfig->set_git_version_hash(git_version_hash);
			logConfig->set_git_version_dirty(git_version_dirty);
			logConfig->set_simulation(_simulation);
		}
		
		////////////////
		// Inputs
		
		// Read vision packets
		vector<const SSL_DetectionFrame *> rawVision;
		int timeout = _framePeriod / 1000;
		while (_syncToVision || _visionSocket->hasPendingDatagrams())
		{
			if (_syncToVision)
			{
				struct pollfd pfd;
				pfd.fd = _visionSocket->socketDescriptor();
				pfd.events = POLLIN;
				if (poll(&pfd, 1, timeout) == 0)
				{
					// Timeout
					break;
				}
				timeout = 0;
			}
			
			string buf;
			unsigned int n = _visionSocket->pendingDatagramSize();
			buf.resize(n);
			_visionSocket->readDatagram(&buf[0], n);
			
			SSL_WrapperPacket *packet = _state.logFrame->add_raw_vision();
			if (!packet->ParseFromString(buf))
			{
				printf("Bad vision packet of %d bytes\n", n);
				continue;
			}
			
			curStatus.lastVisionTime = Utils::timestamp();
			if (packet->has_detection())
			{
				SSL_DetectionFrame *det = packet->mutable_detection();
				
				// Remove balls on the excluded half of the field
				google::protobuf::RepeatedPtrField<SSL_DetectionBall> *balls = det->mutable_balls();
				for (int i = 0; i < balls->size(); ++i)
				{
					float x = balls->Get(i).x();
					//FIXME - OMG too many terms
					if ((!_state.logFrame->use_opponent_half() && ((_defendPlusX && x < 0) || (!_defendPlusX && x > 0))) ||
						(!_state.logFrame->use_our_half() && ((_defendPlusX && x > 0) || (!_defendPlusX && x < 0))))
					{
						balls->SwapElements(i, balls->size() - 1);
						balls->RemoveLast();
						--i;
					}
				}
				
				// Remove robots on the excluded half of the field
				google::protobuf::RepeatedPtrField<SSL_DetectionRobot> *robots[2] =
				{
					det->mutable_robots_yellow(),
					det->mutable_robots_blue()
				};
				
				for (int team = 0; team < 2; ++team)
				{
					for (int i = 0; i < robots[team]->size(); ++i)
					{
						float x = robots[team]->Get(i).x();
						if ((!_state.logFrame->use_opponent_half() && ((_defendPlusX && x < 0) || (!_defendPlusX && x > 0))) ||
							(!_state.logFrame->use_our_half() && ((_defendPlusX && x > 0) || (!_defendPlusX && x < 0))))
						{
							robots[team]->SwapElements(i, robots[team]->size() - 1);
							robots[team]->RemoveLast();
							--i;
						}
					}
				}
				
				rawVision.push_back(det);
			}
		}
		
		// Read referee packets
		while (_refereeSocket->hasPendingDatagrams())
		{
			unsigned int n = _refereeSocket->pendingDatagramSize();
			string str(6, 0);
			_refereeSocket->readDatagram(&str[0], str.size());
			
			// Check the size after receiving to discard bad packets
			if (n != str.size())
			{
				printf("Bad referee packet of %d bytes\n", n);
				continue;
			}
			
			// Log the referee packet, but only use it if external referee is enabled
			curStatus.lastRefereeTime = Utils::timestamp();
			_state.logFrame->add_raw_referee(str);
			
			if (_externalReferee)
			{
				_refereeModule->packet(str);
			}
		}
		
		// Read radio reverse packets
		RadioRx rx;
		if (_radio->receive(&rx))
		{
			_state.logFrame->add_radio_rx()->CopyFrom(rx);
			
			curStatus.lastRadioRxTime = Utils::timestamp();
			
			// Store this packet in the appropriate robot
			unsigned int board = rx.board_id();
			if (board < Num_Shells)
			{
				// We have to copy because the RX packet will survive past this frame
				// but LogFrame will not (the RadioRx in LogFrame will be reused).
				_state.self[board]->radioRx.CopyFrom(rx);
			}
		}
		
		_loopMutex.lock();
		
		_joystick->update();
		
		_modelingModule->run(_blueTeam, rawVision);
		
		// Convert modeling output to team space
		_state.ball.pos = _worldToTeam * _state.ball.pos;
		_state.ball.vel = _worldToTeam.transformDirection(_state.ball.vel);
		_state.ball.accel = _worldToTeam.transformDirection(_state.ball.accel);
		
		BOOST_FOREACH(Robot *robot, _state.self)
		{
			robot->pos = _worldToTeam * robot->pos;
			robot->vel = _worldToTeam.transformDirection(robot->vel);
			robot->angle = Utils::fixAngleDegrees(_teamAngle + robot->angle);
		}

                updateStatusOfBallSensors(state());
		
		BOOST_FOREACH(Robot *robot, _state.opp)
		{
			robot->pos = _worldToTeam * robot->pos;
			robot->vel = _worldToTeam.transformDirection(robot->vel);
			robot->angle = Utils::fixAngleDegrees(_teamAngle + robot->angle);
		}
		
		if (_refereeModule)
		{
			_refereeModule->run();
		}
		
		if (_gameplayModule)
		{
			_gameplayModule->run();
		}

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
		
		_state.logFrame->set_play(_gameplayModule->playName().toStdString());
		
		// Debug layers
		const QStringList &layers = _state.debugLayers();
		BOOST_FOREACH(const QString &str, layers)
		{
			_state.logFrame->add_debug_layers(str.toStdString());
		}
		
		// Our robots
		BOOST_FOREACH(OurRobot *r, _state.self)
		{
			if (r->visible)
			{
				LogFrame::Robot *log = _state.logFrame->add_self();
				*log->mutable_pos() = r->pos;
				*log->mutable_vel() = r->vel;
				*log->mutable_cmd_vel() = r->cmd_vel;
				log->set_cmd_w(r->cmd_w);
				log->set_shell(r->shell());
				log->set_angle(r->angle);
				log->set_ball_sense(r->hasBall);
				
				BOOST_FOREACH(const DebugText &t, r->robotText)
				{
					log->add_text()->CopyFrom(t);
				}
				
				// Copy command trace, converting to uint64
				const vector<void *> &src = r->commandTrace();
				RepeatedField<uint64> *dst = log->mutable_command_trace();
				dst->Reserve(src.size());
				for (unsigned int j = 0; j < src.size(); ++j)
				{
					dst->Add((uint64)src[j]);
				}
			}
		}
		
		// Opponent robots
		BOOST_FOREACH(OpponentRobot *r, _state.opp)
		{
			if (r->visible)
			{
				LogFrame::Robot *log = _state.logFrame->add_opp();
				*log->mutable_pos() = r->pos;
				log->set_shell(r->shell());
				log->set_angle(r->angle);
				*log->mutable_vel() = r->vel;
			}
		}
		
		// Ball
		if (_state.ball.valid)
		{
			LogFrame::Ball *log = _state.logFrame->mutable_ball();
			*log->mutable_pos() = _state.ball.pos;
			*log->mutable_vel() = _state.ball.vel;
		}
		
		////////////////
		// Outputs
		
		// Send motion commands to the robots
		sendRadioData();

		// Write to the log
		_logger.addFrame(_state.logFrame);
		
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
	
	delete _refereeSocket;
	delete _visionSocket;
}

void Processor::sendRadioData()
{
	RadioTx *tx = _state.logFrame->mutable_radio_tx();
	
	// Cycle through reverse IDs for all visible robots
	int giveUp = _reverseId;
	do
	{
		_reverseId = (_reverseId + 1) % Num_Shells;
		if (_state.self[_reverseId]->visible)
		{
			break;
		}
	} while (_reverseId != giveUp);
	tx->set_reverse_board_id(_reverseId);
	
	// Halt overrides normal motion control, but not joystick
	if (!_joystick->autonomous() || _state.gameState.halt())
	{
		// Force all motor speeds to zero
		BOOST_FOREACH(OurRobot *r, _state.self)
		{
			RadioTx::Robot &txRobot = r->radioTx;
			for (int m = 0; m < txRobot.motors_size(); ++m)
			{
				txRobot.set_motors(m, 0);
				txRobot.set_kick(0);
				txRobot.set_roller(0);
			}
		}
	}
	
	// Add RadioTx commands for visible robots and apply joystick input
	BOOST_FOREACH(OurRobot *r, _state.self)
	{
		if (r->visible)
		{
			RadioTx::Robot *txRobot = tx->add_robots();
			
			// Copy motor commands.
			// Even if we are using the joystick, this sets board_id and the
			// number of motors.
			txRobot->CopyFrom(r->radioTx);
			
			if (_manualID >= 0 && (int)r->shell() == _manualID)
			{
				// Drive this robot manually
				_joystick->drive(txRobot);
			}
		}
	}
	
	// Manual driving for invisible robots
	if (_manualID >= 0 && !_state.self[_manualID]->visible && tx->robots_size() < (int)Robots_Per_Team)
	{
		// The manual robot wasn't found by vision/modeling but we have room for it in the packet.
		// This allows us to drive an off-field robot for testing or to drive a robot back onto the field.
		RadioTx::Robot *txRobot = &_state.self[_manualID]->radioTx;
		_joystick->drive(txRobot);
		tx->add_robots()->CopyFrom(*txRobot);
	}

	if (_radio)
	{
		_radio->send(_state.logFrame->radio_tx());
	}
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

	_worldToTeam = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Field_Length / 2.0f));
	_worldToTeam *= Geometry2d::TransformMatrix::rotate(_teamAngle);
}
