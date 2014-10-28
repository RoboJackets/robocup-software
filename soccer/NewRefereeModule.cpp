#include "NewRefereeModule.hpp"

#include <Network.hpp>
#include <multicast.hpp>
#include <Utils.hpp>
#include <unistd.h>
#include <QMutexLocker>
#include <QUdpSocket>
#include <stdexcept>

namespace NewRefereeModuleEnums
{
std::string stringFromStage(Stage s)
{
	switch(s)
	{
	case NORMAL_FIRST_HALF_PRE: return "Normal First Half Prep";
	case NORMAL_FIRST_HALF: return "Normal First Half";
	case NORMAL_HALF_TIME: return "Normal Half Time";
	case NORMAL_SECOND_HALF_PRE: return "Normal Second Half Prep";
	case NORMAL_SECOND_HALF: return "Normal Second Half";
	case EXTRA_TIME_BREAK: return "Extra Time Break";
	case EXTRA_FIRST_HALF_PRE: return "Extra First Half Prep";
	case EXTRA_FIRST_HALF: return "Extra First Half";
	case EXTRA_HALF_TIME: return "Extra Half Time";
	case EXTRA_SECOND_HALF_PRE: return "Extra Second Half Prep";
	case EXTRA_SECOND_HALF: return "Extra Second Half";
	case PENALTY_SHOOTOUT_BREAK: return "Penalty Shootout Break";
	case PENALTY_SHOOTOUT: return "Penalty Shootout";
	case POST_GAME: return "Post Game";
	default: return "";
	}
}

std::string stringFromCommand(Command c)
{
	switch(c)
	{
	case HALT: return "Halt";
	case STOP: return "Stop";
	case NORMAL_START: return "Normal Start";
	case FORCE_START: return "Force Start";
	case PREPARE_KICKOFF_YELLOW: return "Yellow Kickoff Prep";
	case PREPARE_KICKOFF_BLUE: return "Blue Kickoff Prep";
	case PREPARE_PENALTY_YELLOW: return "Yellow Penalty Prep";
	case PREPARE_PENALTY_BLUE: return "Blue Penalty Prep";
	case DIRECT_FREE_YELLOW: return "Direct Yellow Free Kick";
	case DIRECT_FREE_BLUE: return "Direct Blue Free Kick";
	case INDIRECT_FREE_YELLOW: return "Indirect Yellow Free Kick";
	case INDIRECT_FREE_BLUE: return "Indirect Blue Free Kick";
	case TIMEOUT_YELLOW: return "Timeout Yellow";
	case TIMEOUT_BLUE: return "Timeout Blue";
	case GOAL_YELLOW: return "Goal Yellow";
	case GOAL_BLUE: return "Goal Blue";
	default: return "";
	}
}
}

using namespace std;
using namespace NewRefereeModuleEnums;

/// Distance in meters that the ball must travel for a kick to be detected
static const float KickThreshold = 0.150f;
 
/// How many milliseconds the ball must be more than KickThreshold meters away from
/// its position when the referee indicated Ready for us to detect the ball as having been kicked.
static const int KickVerifyTime_ms = 250;

NewRefereeModule::NewRefereeModule(SystemState &state)
	: stage(NORMAL_FIRST_HALF_PRE),
	  command(HALT),
	  _running(false),
	  _state(state)
{
}

NewRefereeModule::~NewRefereeModule()
{
	this->stop();
}

void NewRefereeModule::stop()
{
	_running = false;
	wait();
}

void NewRefereeModule::getPackets(std::vector<NewRefereePacket *> &packets)
{
	_mutex.lock();
	packets = _packets;
	_packets.clear();
	_mutex.unlock();
}

void NewRefereeModule::run()
{
	QUdpSocket socket;

	if(!socket.bind(ProtobufRefereePort, QUdpSocket::ShareAddress))
	{
		throw runtime_error("Can't bind to shared referee port");
	}

	multicast_add(&socket, RefereeAddress);

	_packets.reserve(4);

	_running = true;
	while(_running)
	{
		char buf[65536];

		if(!socket.waitForReadyRead(500))
		{
			continue;
		}

		QHostAddress host;
		quint16 port = 0;
		qint64 size = socket.readDatagram(buf, sizeof(buf), &host, &port);
		if(size < 1)
		{
			fprintf(stderr, "NewRefereeModule: %s/n", (const char *)socket.errorString().toLatin1());
			::usleep(100000);
			continue;
		}

		NewRefereePacket *packet = new NewRefereePacket;
		packet->receivedTime = timestamp();
		this->received_time = packet->receivedTime;
		if(!packet->wrapper.ParseFromArray(buf, size))
		{
			fprintf(stderr, "NewRefereeModule: got bad packet of %d bytes from %s:%d\n", (int)size, (const char *)host.toString().toLatin1(), port);
			fprintf(stderr, "Packet: %s\n", buf);
			fprintf(stderr, "Address: %s\n", RefereeAddress);
			continue;
		}

		_mutex.lock();
		_packets.push_back(packet);

		stage = (Stage)packet->wrapper.stage();
		command = (Command)packet->wrapper.command();
		sent_time = packet->wrapper.packet_timestamp();
		stage_time_left = packet->wrapper.stage_time_left();
		command_counter = packet->wrapper.command_counter();
		command_timestamp = packet->wrapper.command_timestamp();
		yellow_info.ParseRefboxPacket(packet->wrapper.yellow());
		blue_info.ParseRefboxPacket(packet->wrapper.blue());

		_mutex.unlock();

	}
}

void NewRefereeModule::spinKickWatcher() {
	if (_state.ball.valid)
	{
		/// Only run the kick detector when the ball is visible
		switch (_kickDetectState)
		{
			case WaitForReady:
				/// Never kicked and not ready for a restart
				break;
				
			case CapturePosition:
				// Do this in the processing thread
				_readyBallPos = _state.ball.pos;
				_kickDetectState = WaitForKick;
				break;
				
			case WaitForKick:
				if (!_state.ball.pos.nearPoint(_readyBallPos, KickThreshold))
				{
					// The ball appears to have moved
					_kickTime = QTime::currentTime();
					_kickDetectState = VerifyKick;
				}
				break;
				
			case VerifyKick:
				if (_state.ball.pos.nearPoint(_readyBallPos, KickThreshold))
				{
					// The ball is back where it was.  There was probably a vision error.
					_kickDetectState = WaitForKick;
				} else if (_kickTime.msecsTo(QTime::currentTime()) >= KickVerifyTime_ms)
				{
					// The ball has been far enough away for enough time, so call this a kick.
					_kickDetectState = Kicked;
				}
				break;
				
			case Kicked:
				// Stay here until the referee puts us back in Ready
				break;
		}
	}
}

void NewRefereeModule::updateGameState(bool blueTeam) {
	_state.gameState.ourScore = blueTeam ? blue_info.score : yellow_info.score;
	_state.gameState.theirScore = blueTeam ? yellow_info.score : blue_info.score;
	using namespace NewRefereeModuleEnums;
	switch(stage)
	{
	case Stage::NORMAL_FIRST_HALF_PRE:
		_state.gameState.period = GameState::FirstHalf;
		break;
	case Stage::NORMAL_FIRST_HALF:
		_state.gameState.period = GameState::FirstHalf;
		break;
	case Stage::NORMAL_HALF_TIME:
		_state.gameState.period = GameState::Halftime;
		break;
	case Stage::NORMAL_SECOND_HALF_PRE:
		_state.gameState.period = GameState::SecondHalf;
		break;
	case Stage::NORMAL_SECOND_HALF:
		_state.gameState.period = GameState::SecondHalf;
		break;
	case Stage::EXTRA_TIME_BREAK:
		_state.gameState.period = GameState::FirstHalf;
		break;
	case Stage::EXTRA_FIRST_HALF_PRE:
		_state.gameState.period = GameState::Overtime1;
		break;
	case Stage::EXTRA_FIRST_HALF:
		_state.gameState.period = GameState::Overtime1;
		break;
	case Stage::EXTRA_HALF_TIME:
		_state.gameState.period = GameState::Halftime;
		break;
	case Stage::EXTRA_SECOND_HALF_PRE:
		_state.gameState.period = GameState::Overtime2;
		break;
	case Stage::EXTRA_SECOND_HALF:
		_state.gameState.period = GameState::Overtime2;
		break;
	case Stage::PENALTY_SHOOTOUT_BREAK:
		_state.gameState.period = GameState::PenaltyShootout;
		break;
	case Stage::PENALTY_SHOOTOUT:
		_state.gameState.period = GameState::PenaltyShootout;
		break;
	case Stage::POST_GAME:
		_state.gameState.period = GameState::Overtime2;
		break;
	}
	switch(command)
	{
	case Command::HALT:
		_state.gameState.state = GameState::Halt;
		break;
	case Command::STOP:
		_state.gameState.state = GameState::Stop;
		break;
	case Command::NORMAL_START:
		ready();
		break;
	case Command::FORCE_START:
		_state.gameState.state = GameState::Playing;
		break;
	case Command::PREPARE_KICKOFF_YELLOW:
		_state.gameState.state = GameState::Setup;
		_state.gameState.restart = GameState::Kickoff;
		_state.gameState.ourRestart = !blueTeam;
		break;
	case Command::PREPARE_KICKOFF_BLUE:
		_state.gameState.state = GameState::Setup;
		_state.gameState.restart = GameState::Kickoff;
		_state.gameState.ourRestart = blueTeam;
		break;
	case Command::PREPARE_PENALTY_YELLOW:
		_state.gameState.state = GameState::Setup;
		_state.gameState.restart = GameState::Penalty;
		_state.gameState.ourRestart = !blueTeam;
		break;
	case Command::PREPARE_PENALTY_BLUE:
		_state.gameState.state = GameState::Setup;
		_state.gameState.restart = GameState::Penalty;
		_state.gameState.ourRestart = blueTeam;
		break;
	case Command::DIRECT_FREE_YELLOW:
		ready();
		_state.gameState.restart = GameState::Direct;
		_state.gameState.ourRestart = !blueTeam;
		break;
	case Command::DIRECT_FREE_BLUE:
		ready();
		_state.gameState.restart = GameState::Direct;
		_state.gameState.ourRestart = blueTeam;
		break;
	case Command::INDIRECT_FREE_YELLOW:
		ready();
		_state.gameState.restart = GameState::Indirect;
		_state.gameState.ourRestart = !blueTeam;
		break;
	case Command::INDIRECT_FREE_BLUE:
		ready();
		_state.gameState.restart = GameState::Indirect;
		_state.gameState.ourRestart = blueTeam;
		break;
	case Command::TIMEOUT_YELLOW:
		_state.gameState.state = GameState::Halt;
		break;
	case Command::TIMEOUT_BLUE:
		_state.gameState.state = GameState::Halt;
		break;
	case Command::GOAL_YELLOW:
		break;
	case Command::GOAL_BLUE:
		break;
	}

	if(command != prev_command)
		std::cout << "REFEREE: Command = " << stringFromCommand(command) << std::endl;
	prev_command = command;
	if(stage != prev_stage)
		std::cout << "REFEREE: Stage = " << stringFromStage(stage) << std::endl;
	prev_stage = stage;

	if (_state.gameState.state == GameState::Ready && kicked())
	{
		_state.gameState.state = GameState::Playing;
	}
	_state.gameState.OurInfo = blueTeam ? blue_info : yellow_info;
	_state.gameState.TheirInfo = blueTeam ? yellow_info : blue_info;
}

void NewRefereeModule::ready() {
	if(_state.gameState.state == GameState::Stop || _state.gameState.state == GameState::Setup) {
		_state.gameState.state = GameState::Ready;
		_kickDetectState = CapturePosition;
	}
}