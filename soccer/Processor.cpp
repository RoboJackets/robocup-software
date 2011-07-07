// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "Processor.hpp"
#include "VisionReceiver.hpp"
#include "radio/SimRadio.hpp"
#include "radio/USBRadio.hpp"
#include "modeling/BallTracker.hpp"

#include <QMutexLocker>

#include <poll.h>
#include <multicast.hpp>
#include <Constants.hpp>
#include <Network.hpp>
#include <Utils.hpp>
#include <Joystick.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>

#include <motion/MotionControl.hpp>
#include <gameplay/GameplayModule.hpp>
#include <framework/RobotConfig.hpp>
#include <RefereeModule.hpp>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <git_version.h>

REGISTER_CONFIGURABLE(Processor)

using namespace std;
using namespace boost;
using namespace Geometry2d;
using namespace google::protobuf;

static const uint64_t Command_Latency = 0;

RobotConfig *Processor::robotConfig2008;
RobotConfig *Processor::robotConfig2011;
std::vector<RobotStatus*> Processor::robotStatuses; // FIXME: verify that this is correct

void Processor::createConfiguration(Configuration *cfg)
{
	robotConfig2008 = new RobotConfig(cfg, "Rev2008");
	robotConfig2011 = new RobotConfig(cfg, "Rev2011");

	for (size_t s = 0; s<Num_Shells; ++s)
	{
		robotStatuses.push_back(new RobotStatus(cfg, QString("Robot Statuses/Robot %1").arg(s)));
	}
}

Processor::Processor(bool sim)
{
	_running = true;
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

	_ballTracker = make_shared<BallTracker>();
	_refereeModule = make_shared<RefereeModule>(&_state);
	_gameplayModule = make_shared<Gameplay::GameplayModule>(&_state);
}

Processor::~Processor()
{
	stop();
	
	delete _joystick;
	
	//DEBUG - This is unnecessary, but lets us determine which one breaks.
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
	_joystick->reset();
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

void Processor::runModels(const vector<const SSL_DetectionFrame *> &detectionFrames)
{
	vector<BallObservation> ballObservations;
	
	BOOST_FOREACH(const SSL_DetectionFrame* frame, detectionFrames)
	{
		uint64_t time = frame->t_capture() * 1000000;
		
		// Add ball observations
		ballObservations.reserve(ballObservations.size() + frame->balls().size());
		BOOST_FOREACH(const SSL_DetectionBall &ball, frame->balls())
		{
			ballObservations.push_back(BallObservation(_worldToTeam * Point(ball.x() / 1000, ball.y() / 1000), time));
		}
		
		// Add robot observations
		const RepeatedPtrField<SSL_DetectionRobot> &selfRobots = _blueTeam ? frame->robots_blue() : frame->robots_yellow();
		BOOST_FOREACH(const SSL_DetectionRobot &robot, selfRobots)
		{
			float angle = Utils::fixAngleDegrees(robot.orientation() * RadiansToDegrees + _teamAngle);
			RobotObservation obs(_worldToTeam * Point(robot.x() / 1000, robot.y() / 1000), angle, time, frame->frame_number());
			obs.source = frame->camera_id();
			unsigned int id = robot.robot_id();
			if (id < _state.self.size())
			{
				_state.self[id]->filter()->update(&obs);
			}
		}
		
		const RepeatedPtrField<SSL_DetectionRobot> &oppRobots = _blueTeam ? frame->robots_yellow() : frame->robots_blue();
		BOOST_FOREACH(const SSL_DetectionRobot &robot, oppRobots)
		{
			float angle = Utils::fixAngleDegrees(robot.orientation() * RadiansToDegrees + _teamAngle);
			RobotObservation obs(_worldToTeam * Point(robot.x() / 1000, robot.y() / 1000), angle, time, frame->frame_number());
			obs.source = frame->camera_id();
			unsigned int id = robot.robot_id();
			if (id < _state.opp.size())
			{
				_state.opp[id]->filter()->update(&obs);
			}
		}
	}
	
	_ballTracker->run(ballObservations, &_state);
	
	BOOST_FOREACH(Robot *robot, _state.self)
	{
		robot->filter()->predict(_state.logFrame->command_time(), robot);
	}
	
	BOOST_FOREACH(Robot *robot, _state.opp)
	{
		robot->filter()->predict(_state.logFrame->command_time(), robot);
	}
}

void Processor::run()
{
	_refereeSocket = new QUdpSocket;
	
	VisionReceiver vision(_simulation);
	vision.start();
	
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
		_state.logFrame = make_shared<Packet::LogFrame>();
		_state.logFrame->set_command_time(startTime + Command_Latency);
		_state.logFrame->set_use_our_half(_useOurHalf);
		_state.logFrame->set_use_opponent_half(_useOpponentHalf);
		_state.logFrame->set_manual_id(_manualID);
		_state.logFrame->set_blue_team(_blueTeam);
		_state.logFrame->set_defend_plus_x(_defendPlusX);
		
		if (first)
		{
			first = false;
			
			Packet::LogConfig *logConfig = _state.logFrame->mutable_log_config();
			logConfig->set_generator("soccer");
			logConfig->set_git_version_hash(git_version_hash);
			logConfig->set_git_version_dirty(git_version_dirty);
			logConfig->set_simulation(_simulation);
		}
		
		BOOST_FOREACH(OurRobot *robot, _state.self)
		{
			// overall robot config
			switch (robot->hardwareVersion())
			{
			case Packet::RJ2008:
				robot->config = robotConfig2008;
				break;
			case Packet::RJ2011:
				robot->config = robotConfig2011;
				break;
			case Packet::Unknown:
				robot->config = robotConfig2011; // FIXME: defaults to 2011 robots
				break;
			}

			// per-robot configs
			robot->status = robotStatuses.at(robot->shell());
		}

		////////////////
		// Inputs
		
		// Read vision packets
		vector<const SSL_DetectionFrame *> detectionFrames;
		vector<VisionPacket *> visionPackets;
		vision.getPackets(visionPackets);
		BOOST_FOREACH(VisionPacket *packet, visionPackets)
		{
			SSL_WrapperPacket *log = _state.logFrame->add_raw_vision();
			log->CopyFrom(packet->wrapper);
			
			curStatus.lastVisionTime = packet->receivedTime;
			if (packet->wrapper.has_detection())
			{
				SSL_DetectionFrame *det = packet->wrapper.mutable_detection();
				
				//FIXME - Account for network latency
				double rt = packet->receivedTime / 1000000.0;
				det->set_t_capture(rt - det->t_sent() + det->t_capture());
				det->set_t_sent(rt);
				
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
				
				detectionFrames.push_back(det);
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
		_radio->receive();
		BOOST_FOREACH(const Packet::RadioRx &rx, _radio->reversePackets())
		{
			_state.logFrame->add_radio_rx()->CopyFrom(rx);
			
			curStatus.lastRadioRxTime = rx.timestamp();
			
			// Store this packet in the appropriate robot
			unsigned int board = rx.robot_id();
			if (board < Num_Shells)
			{
				// We have to copy because the RX packet will survive past this frame
				// but LogFrame will not (the RadioRx in LogFrame will be reused).
				_state.self[board]->radioRx.CopyFrom(rx);
			}
		}
		_radio->clear();
		
		_loopMutex.lock();
		
		_joystick->update();
		
		runModels(detectionFrames);

		if (_refereeModule)
		{
			_refereeModule->run();
		}
		
		if (_gameplayModule)
		{
			_gameplayModule->run();
		}

		// Run velocity controllers
		BOOST_FOREACH(OurRobot *robot, _state.self)
		{
			if (robot->visible)
			{
				if ((_manualID >= 0 && (int)robot->shell() == _manualID) || !_joystick->autonomous() || _state.gameState.halt())
				{
					robot->motionControl()->stopped();
				}
				
				robot->motionControl()->run();
			}
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
				r->addStatusText();
				
				Packet::LogFrame::Robot *log = _state.logFrame->add_self();
				*log->mutable_pos() = r->pos;
				*log->mutable_vel() = r->vel;
				*log->mutable_cmd_vel() = r->cmd_vel;
				log->set_cmd_w(r->cmd_w);
				log->set_shell(r->shell());
				log->set_angle(r->angle);
				
				if (r->radioRx.has_kicker_voltage())
				{
					log->set_kicker_voltage(r->radioRx.kicker_voltage());
				}
				
				if (r->radioRx.has_kicker_status())
				{
					log->set_charged(r->radioRx.kicker_status() & 0x01);
					log->set_kicker_works(!(r->radioRx.kicker_status() & 0x90));
				}
				
				if (r->radioRx.has_ball_sense_status())
				{
					log->set_ball_sense_status(r->radioRx.ball_sense_status());
				}
				
				if (r->radioRx.has_battery())
				{
					log->set_battery_voltage(r->radioRx.battery());
				}
				
				log->mutable_motor_status()->Clear();
				log->mutable_motor_status()->MergeFrom(r->radioRx.motor_status());
				
				if (r->radioRx.has_quaternion())
				{
					log->mutable_quaternion()->Clear();
					log->mutable_quaternion()->MergeFrom(r->radioRx.quaternion());
				} else {
					log->clear_quaternion();
				}
				
				BOOST_FOREACH(const Packet::DebugText &t, r->robotText)
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
				Packet::LogFrame::Robot *log = _state.logFrame->add_opp();
				*log->mutable_pos() = r->pos;
				log->set_shell(r->shell());
				log->set_angle(r->angle);
				*log->mutable_vel() = r->vel;
			}
		}
		
		// Ball
		if (_state.ball.valid)
		{
			Packet::LogFrame::Ball *log = _state.logFrame->mutable_ball();
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
			// Use system usleep, not QThread::usleep.
			//
			// QThread::usleep uses pthread_cond_wait which sometimes fails to unblock.
			// This seems to depend on how many threads are blocked.
			::usleep(_framePeriod - lastFrameTime);
		} else {
//			printf("Processor took too long: %d us\n", lastFrameTime);
		}
	}
	
	vision.stop();
	
	delete _refereeSocket;
}

void Processor::sendRadioData()
{
	Packet::RadioTx *tx = _state.logFrame->mutable_radio_tx();
	
	// Halt overrides normal motion control, but not joystick
	if (!_joystick->autonomous() || _state.gameState.halt())
	{
		// Force all motor speeds to zero
		BOOST_FOREACH(OurRobot *r, _state.self)
		{
			Packet::RadioTx::Robot &txRobot = r->radioTx;
			txRobot.set_body_x(0);
			txRobot.set_body_y(0);
			txRobot.set_body_w(0);
			txRobot.set_kick(0);
			txRobot.set_dribbler(0);
		}
	}
	
	// Add RadioTx commands for visible robots and apply joystick input
	BOOST_FOREACH(OurRobot *r, _state.self)
	{
		if (r->visible)
		{
			Packet::RadioTx::Robot *txRobot = tx->add_robots();
			
			// Copy motor commands.
			// Even if we are using the joystick, this sets robot_id and the
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
		Packet::RadioTx::Robot *txRobot = &_state.self[_manualID]->radioTx;
		_joystick->drive(txRobot);
		tx->add_robots()->CopyFrom(*txRobot);
	}

	if (_radio)
	{
		_radio->send(*_state.logFrame->mutable_radio_tx());
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
