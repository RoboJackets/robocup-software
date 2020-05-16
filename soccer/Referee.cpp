#include "Referee.hpp"

#include <unistd.h>

#include <Network.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <multicast.hpp>
#include <stdexcept>

#include "Constants.hpp"
#include "RefereeEnums.hpp"

using RefereeModuleEnums::Command;
using RefereeModuleEnums::Stage;

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
    : stage_(Stage::NORMAL_FIRST_HALF_PRE),
      command_(Command::HALT),
      sent_time{},
      received_time{},
      stage_time_left{},
      command_counter{},
      yellow_info{},
      blue_info{},
      _kickDetectState{},
      _readyBallPos{},
      _kickTime{},
      _mutex{},
      _packets{},
      _context(context),
      prev_command_{},
      prev_stage_{},
      ballPlacementX{},
      ballPlacementY{},
      _recv_buffer{},
      _asio_socket{_io_service} {}

void Referee::start() {
    setupRefereeMulticast();
    startReceive();
}

void Referee::getPackets(std::vector<RefereePacket>& packets) {
    std::lock_guard<std::mutex> lock(_mutex);
    packets = _packets;
    _packets.clear();
}

void Referee::startReceive() {
    // Set a receive callback
    _asio_socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _sender_endpoint,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receivePacket(error, num_bytes);
            startReceive();
        });
}

void Referee::receivePacket(const boost::system::error_code& error,
                            size_t num_bytes) {
    if (error != boost::system::errc::success) {
        std::cerr << "Error receiving: " << error << " in " __FILE__
                  << std::endl;
        return;
    }

    if (!_useExternalRef) {
        return;
    }

    RefereePacket packet{};
    packet.receivedTime = RJ::now();
    this->received_time = packet.receivedTime;

    if (!packet.wrapper.ParseFromArray(_recv_buffer.data(), num_bytes)) {
        std::cerr << "NewRefereeModule: got bad packet of " << num_bytes
                  << " bytes from " << _sender_endpoint << std::endl;
        std::cerr << "Address: " << &RefereeAddress << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock{_mutex};
    _packets.push_back(packet);

    stage_ = static_cast<Stage>(packet.wrapper.stage());
    command_ = static_cast<Command>(packet.wrapper.command());
    sent_time =
        RJ::Time(std::chrono::microseconds(packet.wrapper.packet_timestamp()));
    stage_time_left =
        std::chrono::milliseconds(packet.wrapper.stage_time_left());
    command_counter = packet.wrapper.command_counter();
    command_timestamp =
        RJ::Time(std::chrono::microseconds(packet.wrapper.command_timestamp()));
    yellow_info.ParseRefboxPacket(packet.wrapper.yellow());
    blue_info.ParseRefboxPacket(packet.wrapper.blue());
    ballPlacementX = packet.wrapper.designated_position().x();
    ballPlacementY = packet.wrapper.designated_position().y();

    // If we have no name, we're using a default config and there's no
    // sense trying to match the referee's output (because chances are
    // everything is just on default configuration with no names, so
    // more than one team/soccer instance will be trying to use the same
    // color)
    if (Team_Name_Lower.length() > 0) {
        // We only want to change teams if we get something that
        // actually matches our team name (either yellow or blue).
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
}

void Referee::setupRefereeMulticast() {
    const auto any_address = boost::asio::ip::address_v4::any();
    boost::asio::ip::udp::endpoint listen_endpoint{any_address,
                                                   ProtobufRefereePort};

    _asio_socket.open(listen_endpoint.protocol());
    _asio_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    try {
        _asio_socket.bind(listen_endpoint);
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Failed to bind to shared referee port");
    }
    // Join multicast group
    const boost::asio::ip::address multicast_address =
        boost::asio::ip::address::from_string(RefereeAddress);
    _asio_socket.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));
}

void Referee::run() {
    _io_service.poll();
    spinKickWatcher(_context->world_state.ball);
    update();
}

void Referee::overrideTeam(bool isBlue) { _game_state.blueTeam = isBlue; }

void Referee::spinKickWatcher(const BallState& ball) {
    /// Only run the kick detector when the ball is visible
    if (!ball.visible) {
        return;
    }

    switch (_kickDetectState) {
        case WaitForReady:
            /// Never kicked and not ready for a restart
            break;

        case CapturePosition:
            // Do this in the processing thread
            _readyBallPos = ball.position;
            _kickDetectState = WaitForKick;
            break;

        case WaitForKick:
            if (ball.position.nearPoint(_readyBallPos, KickThreshold)) {
                return;
            }
            // The ball appears to have moved
            _kickTime = RJ::Time();
            _kickDetectState = VerifyKick;
            break;

        case VerifyKick: {
            const auto ms_elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    RJ::Time() - _kickTime)
                    .count();
            if (ball.position.nearPoint(_readyBallPos, KickThreshold)) {
                // The ball is back where it was.  There was probably a
                // vision error.
                _kickDetectState = WaitForKick;
            } else if (ms_elapsed >= KickVerifyTime_ms) {
                // The ball has been far enough away for enough time, so
                // call this a kick.
                _kickDetectState = Kicked;
            }
            break;
        }
        case Kicked:
            // Stay here until the referee puts us back in Ready
            break;
    }
}

void Referee::update() {
    _game_state = updateGameState(command_);
    _context->game_state = _game_state;

    switch (command_) {
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

    if (command_ != prev_command_) {
        std::cout << "REFEREE: Command = " << stringFromCommand(command_)
                  << std::endl;
        prev_command_ = command_;
    }

    if (stage_ != prev_stage_) {
        std::cout << "REFEREE: Stage = " << stringFromStage(stage_)
                  << std::endl;
        prev_stage_ = stage_;
    }
}

GameState Referee::updateGameState(Command command) const {
    using namespace RefereeModuleEnums;

    const GameState::Period period = [stage =
                                          this->stage_]() -> GameState::Period {
        switch (stage) {
            case Stage::NORMAL_FIRST_HALF_PRE:
            case Stage::NORMAL_FIRST_HALF:
                return GameState::FirstHalf;
            case Stage::NORMAL_HALF_TIME:
                return GameState::Halftime;
            case Stage::NORMAL_SECOND_HALF_PRE:
            case Stage::NORMAL_SECOND_HALF:
                return GameState::SecondHalf;
            case Stage::EXTRA_TIME_BREAK:
                return GameState::FirstHalf;
            case Stage::EXTRA_FIRST_HALF_PRE:
            case Stage::EXTRA_FIRST_HALF:
                return GameState::Overtime1;
            case Stage::EXTRA_HALF_TIME:
                return GameState::Halftime;
            case Stage::EXTRA_SECOND_HALF_PRE:
            case Stage::EXTRA_SECOND_HALF:
                return GameState::Overtime2;
            case Stage::PENALTY_SHOOTOUT_BREAK:
            case Stage::PENALTY_SHOOTOUT:
                return GameState::PenaltyShootout;
            case Stage::POST_GAME:
                return GameState::Overtime2;
            default:
                return GameState::FirstHalf;
        }
    }();

    GameState::State state = _game_state.state;
    GameState::Restart restart = _game_state.restart;
    bool our_restart = _game_state.ourRestart;
    Geometry2d::Point ball_placement_point = _game_state.ballPlacementPoint;

    const bool blue_team = _game_state.blueTeam;

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
        case Command::TIMEOUT_BLUE:
            state = GameState::Halt;
            break;
        case Command::GOAL_YELLOW:
        case Command::GOAL_BLUE:
            break;
        case Command::BALL_PLACEMENT_YELLOW:
            state = GameState::Stop;
            restart = GameState::Placement;
            our_restart = !blue_team;
            ball_placement_point = GameState::convertToBallPlacementPoint(
                ballPlacementX, ballPlacementY);
            break;
        case Command::BALL_PLACEMENT_BLUE:
            state = GameState::Stop;
            restart = GameState::Placement;
            our_restart = blue_team;
            ball_placement_point = GameState::convertToBallPlacementPoint(
                ballPlacementX, ballPlacementY);
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
                     _game_state.secondsRemaining,
                     our_info,
                     their_info,
                     blue_team,
                     ball_placement_point,
                     _game_state.defendPlusX};
}
