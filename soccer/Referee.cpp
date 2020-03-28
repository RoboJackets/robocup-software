#include "Referee.hpp"

#include <unistd.h>

#include <Network.hpp>
#include <QMutexLocker>
#include <QUdpSocket>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <multicast.hpp>
#include <stdexcept>

#include "Constants.hpp"

namespace RefreeModuleEnums {
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
}  // namespace RefreeModuleEnums

using namespace RefreeModuleEnums;

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

Referee::Referee(Context* const context)
    : stage(NORMAL_FIRST_HALF_PRE),
      command(HALT),
      _running(false),
      _context(context) {}

Referee::~Referee() { stop(); }

void Referee::stop() {
    _running = false;
    wait();
}

void Referee::getPackets(std::vector<RefereePacket*>& packets) {
    _mutex.lock();
    packets = _packets;
    _packets.clear();
    _mutex.unlock();
}

void Referee::run() {
    QUdpSocket socket;

    if (!socket.bind(ProtobufRefereePort, QUdpSocket::ShareAddress)) {
        throw std::runtime_error("Can't bind to shared referee port");
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

        RefereePacket* packet = new RefereePacket;
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

        // If we have no name, we're using a default config and there's no
        // sense trying to match the referee's output (because chances are
        // everything is just on default configuration with no names, so more
        // than one team/soccer instance will be trying to use the same color)
        if (Team_Name_Lower.length() > 0) {
            // We only want to change teams if we get something that actually
            // matches our team name (either yellow or blue).
            // Otherwise, keep the current color.
            if (boost::iequals(yellow_info.name, Team_Name_Lower)) {
                blueTeam(false);
                _isRefControlled = true;
            } else if (boost::iequals(blue_info.name, Team_Name_Lower)) {
                blueTeam(true);
                _isRefControlled = true;
            } else {
                _isRefControlled = false;
            }
        }

        _mutex.unlock();
    }
}

void Referee::overrideTeam(bool isBlue) {
    _context->game_state.blueTeam = isBlue;
}

void Referee::spin() {
    spinKickWatcher(_context->state);
    update();
}

void Referee::spinKickWatcher(const SystemState& system_state) {
    if (system_state.ball.valid) {
        /// Only run the kick detector when the ball is visible
        switch (_kickDetectState) {
            case WaitForReady:
                /// Never kicked and not ready for a restart
                break;

            case CapturePosition:
                // Do this in the processing thread
                _readyBallPos = system_state.ball.pos;
                _kickDetectState = WaitForKick;
                break;

            case WaitForKick:
                if (!system_state.ball.pos.nearPoint(_readyBallPos,
                                                     KickThreshold)) {
                    // The ball appears to have moved
                    _kickTime = QTime::currentTime();
                    _kickDetectState = VerifyKick;
                }
                break;

            case VerifyKick:
                if (system_state.ball.pos.nearPoint(_readyBallPos,
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

void Referee::update() {
    _context->game_state = updateGameState(_context->game_state);

    switch (command) {
        case Command::NORMAL_START:
        case Command::DIRECT_FREE_YELLOW:
        case Command::DIRECT_FREE_BLUE:
        case Command::INDIRECT_FREE_YELLOW:
        case Command::INDIRECT_FREE_BLUE: {
            if (_context->game_state.state == GameState::Stop ||
                _context->game_state.state == GameState::Setup) {
                _context->game_state.state = GameState::Ready;
                _kickDetectState = CapturePosition;
            }
        }
        default:
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
}

GameState Referee::updateGameState(const GameState& game_state) const {
    using namespace RefreeModuleEnums;

    const GameState::Period period = [stage =
                                          this->stage]() -> GameState::Period {
        switch (stage) {
            case Stage::NORMAL_FIRST_HALF_PRE:
                return GameState::FirstHalf;
            case Stage::NORMAL_FIRST_HALF:
                return GameState::FirstHalf;
            case Stage::NORMAL_HALF_TIME:
                return GameState::Halftime;
            case Stage::NORMAL_SECOND_HALF_PRE:
                return GameState::SecondHalf;
            case Stage::NORMAL_SECOND_HALF:
                return GameState::SecondHalf;
            case Stage::EXTRA_TIME_BREAK:
                return GameState::FirstHalf;
            case Stage::EXTRA_FIRST_HALF_PRE:
                return GameState::Overtime1;
            case Stage::EXTRA_FIRST_HALF:
                return GameState::Overtime1;
            case Stage::EXTRA_HALF_TIME:
                return GameState::Halftime;
            case Stage::EXTRA_SECOND_HALF_PRE:
                return GameState::Overtime2;
            case Stage::EXTRA_SECOND_HALF:
                return GameState::Overtime2;
            case Stage::PENALTY_SHOOTOUT_BREAK:
                return GameState::PenaltyShootout;
            case Stage::PENALTY_SHOOTOUT:
                return GameState::PenaltyShootout;
            case Stage::POST_GAME:
                return GameState::Overtime2;
            default:
                return GameState::FirstHalf;
        };
    }();

    GameState::State state = game_state.state;
    GameState::Restart restart = game_state.restart;
    bool our_restart = game_state.ourRestart;
    Geometry2d::Point ball_placement_point = game_state.ballPlacementPoint;

    const bool blue_team = game_state.blueTeam;

    switch (command) {
        case Command::HALT:
            state = GameState::Halt;

            if constexpr (CancelBallPlaceOnHalt) {
                restart = GameState::None;
                our_restart = false;
            }
            break;
        case Command::STOP:
            state = GameState::Stop;
            break;
        case Command::NORMAL_START:
            break;
        case Command::FORCE_START:
            state = GameState::Playing;
            our_restart = false;
            restart = GameState::None;
            break;
        case Command::PREPARE_KICKOFF_YELLOW:
            state = GameState::Setup;
            restart = GameState::Kickoff;
            our_restart = !blue_team;
            break;
        case Command::PREPARE_KICKOFF_BLUE:
            state = GameState::Setup;
            restart = GameState::Kickoff;
            our_restart = blue_team;
            break;
        case Command::PREPARE_PENALTY_YELLOW:
            state = GameState::Setup;
            restart = GameState::Penalty;
            our_restart = !blue_team;
            break;
        case Command::PREPARE_PENALTY_BLUE:
            state = GameState::Setup;
            restart = GameState::Penalty;
            our_restart = blue_team;
            break;
        case Command::DIRECT_FREE_YELLOW:
            restart = GameState::Direct;
            our_restart = !blue_team;
            break;
        case Command::DIRECT_FREE_BLUE:
            restart = GameState::Direct;
            our_restart = blue_team;
            break;
        case Command::INDIRECT_FREE_YELLOW:
            restart = GameState::Indirect;
            our_restart = !blue_team;
            break;
        case Command::INDIRECT_FREE_BLUE:
            restart = GameState::Indirect;
            our_restart = blue_team;
            break;
        case Command::TIMEOUT_YELLOW:
            state = GameState::Halt;
            break;
        case Command::TIMEOUT_BLUE:
            state = GameState::Halt;
            break;
        case Command::GOAL_YELLOW:
            break;
        case Command::GOAL_BLUE:
            break;
        case Command::BALL_PLACEMENT_YELLOW:
            state = GameState::Stop;
            restart = GameState::Placement;
            our_restart = !blue_team;
            ball_placement_point = GameState::convertToBallPlacementPoint(
                ballPlacementx, ballPlacementy);
            break;
        case Command::BALL_PLACEMENT_BLUE:
            state = GameState::Stop;
            restart = GameState::Placement;
            our_restart = blue_team;
            ball_placement_point = GameState::convertToBallPlacementPoint(
                ballPlacementx, ballPlacementy);
            break;
    }

    if (state == GameState::Ready && kicked()) {
        state = GameState::Playing;
    }

    const int our_score = blue_team ? blue_info.score : yellow_info.score;
    const int their_score = blue_team ? yellow_info.score : blue_info.score;

    const auto our_info = blue_team ? blue_info : yellow_info;
    const auto their_info = blue_team ? yellow_info : blue_info;

    return GameState{period,
                     state,
                     restart,
                     our_restart,
                     our_score,
                     their_score,
                     game_state.secondsRemaining,
                     our_info,
                     their_info,
                     blue_team,
                     ball_placement_point,
                     game_state.defendPlusX};
}
