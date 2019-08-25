#include "NewRefereeModule.hpp"

#include "Constants.hpp"

#include <Network.hpp>
#include <multicast.hpp>
#include <Utils.hpp>
#include <unistd.h>
#include <QMutexLocker>
#include <QUdpSocket>
#include <stdexcept>

namespace NewRefereeModuleEnums {
std::string stringFromStage(Stage s) {
    switch (s) {
        case NORMAL_FIRST_HALF_PRE:
            return "Normal First Half Prep";
        case NORMAL_FIRST_HALF:
            return "Normal First Half";
        case NORMAL_HALF_TIME:
            return "Normal Half Time";
        case NORMAL_SECOND_HALF_PRE:
            return "Normal Second Half Prep";
        case NORMAL_SECOND_HALF:
            return "Normal Second Half";
        case EXTRA_TIME_BREAK:
            return "Extra Time Break";
        case EXTRA_FIRST_HALF_PRE:
            return "Extra First Half Prep";
        case EXTRA_FIRST_HALF:
            return "Extra First Half";
        case EXTRA_HALF_TIME:
            return "Extra Half Time";
        case EXTRA_SECOND_HALF_PRE:
            return "Extra Second Half Prep";
        case EXTRA_SECOND_HALF:
            return "Extra Second Half";
        case PENALTY_SHOOTOUT_BREAK:
            return "Penalty Shootout Break";
        case PENALTY_SHOOTOUT:
            return "Penalty Shootout";
        case POST_GAME:
            return "Post Game";
        default:
            return "";
    }
}

std::string stringFromCommand(Command c) {
    switch (c) {
        case HALT:
            return "Halt";
        case STOP:
            return "Stop";
        case NORMAL_START:
            return "Normal Start";
        case FORCE_START:
            return "Force Start";
        case PREPARE_KICKOFF_YELLOW:
            return "Yellow Kickoff Prep";
        case PREPARE_KICKOFF_BLUE:
            return "Blue Kickoff Prep";
        case PREPARE_PENALTY_YELLOW:
            return "Yellow Penalty Prep";
        case PREPARE_PENALTY_BLUE:
            return "Blue Penalty Prep";
        case DIRECT_FREE_YELLOW:
            return "Direct Yellow Free Kick";
        case DIRECT_FREE_BLUE:
            return "Direct Blue Free Kick";
        case INDIRECT_FREE_YELLOW:
            return "Indirect Yellow Free Kick";
        case INDIRECT_FREE_BLUE:
            return "Indirect Blue Free Kick";
        case TIMEOUT_YELLOW:
            return "Timeout Yellow";
        case TIMEOUT_BLUE:
            return "Timeout Blue";
        case GOAL_YELLOW:
            return "Goal Yellow";
        case GOAL_BLUE:
            return "Goal Blue";
        case BALL_PLACEMENT_YELLOW:
            return "Ball Placement Yellow";
        case BALL_PLACEMENT_BLUE:
            return "Ball Placement Blue";
        default:
            return "";
    }
}
}

using namespace std;
using namespace NewRefereeModuleEnums;

/// Distance in meters that the ball must travel for a kick to be detected
static const float KickThreshold = Ball_Radius * 3;

/// How many milliseconds the ball must be more than KickThreshold meters away
/// from its position when the referee indicated Ready for us to detect the ball
/// as having been kicked.
static const int KickVerifyTime_ms = 250;

// Whether we cancel ball placement on a halt.
// If we want ball placement to continue after
// the ref halts/stops, make this false
static const bool CancelBallPlaceOnHalt = true;

NewRefereeModule::NewRefereeModule(Context* const context)
    : stage(NORMAL_FIRST_HALF_PRE),
      command(HALT),
      _running(false),
      _context(context) {}

NewRefereeModule::~NewRefereeModule() { this->stop(); }

void NewRefereeModule::stop() {
    _running = false;
    wait();
}

void NewRefereeModule::getPackets(std::vector<NewRefereePacket*>& packets) {
    _mutex.lock();
    packets = _packets;
    _packets.clear();
    _mutex.unlock();
}

void NewRefereeModule::run() {
    QUdpSocket socket;

    if (!socket.bind(ProtobufRefereePort, QUdpSocket::ShareAddress)) {
        throw runtime_error("Can't bind to shared referee port");
    }

    multicast_add(&socket, RefereeAddress);

    _packets.reserve(4);

    _running = true;
    while (_running) {
        if (!_useExternalRef) continue;

        char buf[65536];

        if (!socket.waitForReadyRead(500)) {
            continue;
        }

        QHostAddress host;
        quint16 port = 0;
        qint64 size = socket.readDatagram(buf, sizeof(buf), &host, &port);
        if (size < 1) {
            fprintf(stderr, "NewRefereeModule: %s/n",
                    (const char*)socket.errorString().toLatin1());
            ::usleep(100000);
            continue;
        }

        NewRefereePacket* packet = new NewRefereePacket;
        packet->receivedTime = RJ::now();
        this->received_time = packet->receivedTime;
        if (!packet->wrapper.ParseFromArray(buf, size)) {
            fprintf(stderr,
                    "NewRefereeModule: got bad packet of %d bytes from %s:%d\n",
                    (int)size, (const char*)host.toString().toLatin1(), port);
            fprintf(stderr, "Packet: %s\n", buf);
            fprintf(stderr, "Address: %s\n", RefereeAddress);
            continue;
        }

        _mutex.lock();
        _packets.push_back(packet);

        stage = (Stage)packet->wrapper.stage();
        command = (Command)packet->wrapper.command();
        sent_time = RJ::Time(
            std::chrono::microseconds(packet->wrapper.packet_timestamp()));
        stage_time_left =
            std::chrono::milliseconds(packet->wrapper.stage_time_left());
        command_counter = packet->wrapper.command_counter();
        command_timestamp = RJ::Time(
            std::chrono::microseconds(packet->wrapper.command_timestamp()));
        yellow_info.ParseRefboxPacket(packet->wrapper.yellow());
        blue_info.ParseRefboxPacket(packet->wrapper.blue());
        ballPlacementx = packet->wrapper.designated_position().x();
        ballPlacementy = packet->wrapper.designated_position().y();

        std::string yellow_name = yellow_info.name;
        for (char& letter : yellow_name) {
            letter = tolower(letter);
        }

        blueTeam(yellow_name != Team_Name_Lower);

        _mutex.unlock();
    }
}

void NewRefereeModule::spinKickWatcher() {
    if (_context->state.ball.valid) {
        /// Only run the kick detector when the ball is visible
        switch (_kickDetectState) {
            case WaitForReady:
                /// Never kicked and not ready for a restart
                break;

            case CapturePosition:
                // Do this in the processing thread
                _readyBallPos = _context->state.ball.pos;
                _kickDetectState = WaitForKick;
                break;

            case WaitForKick:
                if (!_context->state.ball.pos.nearPoint(_readyBallPos,
                                                        KickThreshold)) {
                    // The ball appears to have moved
                    _kickTime = QTime::currentTime();
                    _kickDetectState = VerifyKick;
                }
                break;

            case VerifyKick:
                if (_context->state.ball.pos.nearPoint(_readyBallPos,
                                                       KickThreshold)) {
                    // The ball is back where it was.  There was probably a
                    // vision error.
                    _kickDetectState = WaitForKick;
                } else if (_kickTime.msecsTo(QTime::currentTime()) >=
                           KickVerifyTime_ms) {
                    // The ball has been far enough away for enough time, so
                    // call this a kick.
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
    _context->game_state.ourScore =
        blueTeam ? blue_info.score : yellow_info.score;
    _context->game_state.theirScore =
        blueTeam ? yellow_info.score : blue_info.score;
    using namespace NewRefereeModuleEnums;
    switch (stage) {
        case Stage::NORMAL_FIRST_HALF_PRE:
            _context->game_state.period = GameState::FirstHalf;
            break;
        case Stage::NORMAL_FIRST_HALF:
            _context->game_state.period = GameState::FirstHalf;
            break;
        case Stage::NORMAL_HALF_TIME:
            _context->game_state.period = GameState::Halftime;
            break;
        case Stage::NORMAL_SECOND_HALF_PRE:
            _context->game_state.period = GameState::SecondHalf;
            break;
        case Stage::NORMAL_SECOND_HALF:
            _context->game_state.period = GameState::SecondHalf;
            break;
        case Stage::EXTRA_TIME_BREAK:
            _context->game_state.period = GameState::FirstHalf;
            break;
        case Stage::EXTRA_FIRST_HALF_PRE:
            _context->game_state.period = GameState::Overtime1;
            break;
        case Stage::EXTRA_FIRST_HALF:
            _context->game_state.period = GameState::Overtime1;
            break;
        case Stage::EXTRA_HALF_TIME:
            _context->game_state.period = GameState::Halftime;
            break;
        case Stage::EXTRA_SECOND_HALF_PRE:
            _context->game_state.period = GameState::Overtime2;
            break;
        case Stage::EXTRA_SECOND_HALF:
            _context->game_state.period = GameState::Overtime2;
            break;
        case Stage::PENALTY_SHOOTOUT_BREAK:
            _context->game_state.period = GameState::PenaltyShootout;
            break;
        case Stage::PENALTY_SHOOTOUT:
            _context->game_state.period = GameState::PenaltyShootout;
            break;
        case Stage::POST_GAME:
            _context->game_state.period = GameState::Overtime2;
            break;
    }
    switch (command) {
        case Command::HALT:
            _context->game_state.state = GameState::Halt;

            if (CancelBallPlaceOnHalt) {
                _context->game_state.restart = GameState::None;
                _context->game_state.ourRestart = false;
            }
            break;
        case Command::STOP:
            _context->game_state.state = GameState::Stop;
            break;
        case Command::NORMAL_START:
            ready();
            break;
        case Command::FORCE_START:
            _context->game_state.state = GameState::Playing;
            _context->game_state.ourRestart = false;
            _context->game_state.restart = GameState::None;
            break;
        case Command::PREPARE_KICKOFF_YELLOW:
            _context->game_state.state = GameState::Setup;
            _context->game_state.restart = GameState::Kickoff;
            _context->game_state.ourRestart = !blueTeam;
            break;
        case Command::PREPARE_KICKOFF_BLUE:
            _context->game_state.state = GameState::Setup;
            _context->game_state.restart = GameState::Kickoff;
            _context->game_state.ourRestart = blueTeam;
            break;
        case Command::PREPARE_PENALTY_YELLOW:
            _context->game_state.state = GameState::Setup;
            _context->game_state.restart = GameState::Penalty;
            _context->game_state.ourRestart = !blueTeam;
            break;
        case Command::PREPARE_PENALTY_BLUE:
            _context->game_state.state = GameState::Setup;
            _context->game_state.restart = GameState::Penalty;
            _context->game_state.ourRestart = blueTeam;
            break;
        case Command::DIRECT_FREE_YELLOW:
            ready();
            _context->game_state.restart = GameState::Direct;
            _context->game_state.ourRestart = !blueTeam;
            break;
        case Command::DIRECT_FREE_BLUE:
            ready();
            _context->game_state.restart = GameState::Direct;
            _context->game_state.ourRestart = blueTeam;
            break;
        case Command::INDIRECT_FREE_YELLOW:
            ready();
            _context->game_state.restart = GameState::Indirect;
            _context->game_state.ourRestart = !blueTeam;
            break;
        case Command::INDIRECT_FREE_BLUE:
            ready();
            _context->game_state.restart = GameState::Indirect;
            _context->game_state.ourRestart = blueTeam;
            break;
        case Command::TIMEOUT_YELLOW:
            _context->game_state.state = GameState::Halt;
            break;
        case Command::TIMEOUT_BLUE:
            _context->game_state.state = GameState::Halt;
            break;
        case Command::GOAL_YELLOW:
            break;
        case Command::GOAL_BLUE:
            break;
        case Command::BALL_PLACEMENT_YELLOW:
            _context->game_state.state = GameState::Stop;
            _context->game_state.restart = GameState::Placement;
            _context->game_state.ourRestart = !blueTeam;
            _context->game_state.setBallPlacementPoint(ballPlacementx,
                                                       ballPlacementy);
            break;
        case Command::BALL_PLACEMENT_BLUE:
            _context->game_state.state = GameState::Stop;
            _context->game_state.restart = GameState::Placement;
            _context->game_state.ourRestart = blueTeam;
            _context->game_state.setBallPlacementPoint(ballPlacementx,
                                                       ballPlacementy);
            break;
    }

    if (command != prev_command) {
        std::cout << "REFEREE: Command = " << stringFromCommand(command)
                  << std::endl;
        prev_command = command;
    }

    if (stage != prev_stage) {
        std::cout << "REFEREE: Stage = " << stringFromStage(stage) << std::endl;
        prev_stage = stage;
    }

    if (_context->game_state.state == GameState::Ready && kicked()) {
        _context->game_state.state = GameState::Playing;
    }

    _context->game_state.OurInfo = blueTeam ? blue_info : yellow_info;
    _context->game_state.TheirInfo = blueTeam ? yellow_info : blue_info;
}

void NewRefereeModule::ready() {
    if (_context->game_state.state == GameState::Stop ||
        _context->game_state.state == GameState::Setup) {
        _context->game_state.state = GameState::Ready;
        _kickDetectState = CapturePosition;
    }
}
