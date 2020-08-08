#include "external_referee.hpp"

#include <stdexcept>

#include <boost/algorithm/string/predicate.hpp>

#include <rj_common/network.hpp>
#include <rj_common/referee_enums.hpp>
#include <rj_common/utils.hpp>
#include <rj_common/multicast.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_utils/logging.hpp>
#include <unistd.h>

#include "world_state.hpp"

namespace referee {

using RefereeModuleEnums::Command;
using RefereeModuleEnums::Stage;

/// Distance in meters that the ball must travel for a kick to be detected
static const float kKickThreshold = kBallRadius * 3;

/// How many milliseconds the ball must be more than KickThreshold meters away
/// from its position when the referee indicated Ready for us to detect the ball
/// as having been kicked.
static const int kKickVerifyTimeMs = 250;

// Whether we cancel ball placement on a halt.
// If we want ball placement to continue after
// the ref halts/stops, make this false
static const bool kCancelBallPlaceOnHalt = true;

DEFINE_STRING(kRefereeParamModule, team_name, "RoboJackets",
              "The team name we should use when automatically assigning team "
              "colors from referee");

ExternalReferee::ExternalReferee() : RefereeBase{"external_referee"}, asio_socket_{io_service_} {
    set_team_name(PARAM_team_name);

    raw_ref_pub_ = create_publisher<RawProtobufMsg>(referee::topics::kRefereeRawPub, 10);

    network_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { this->update(); });

    // Set up networking for external referee packets
    setup_referee_multicast();
    start_receive();
}

void ExternalReferee::start_receive() {
    // Set a receive callback
    asio_socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), sender_endpoint_,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receive_packet(error, num_bytes);
            start_receive();
        });
}

void ExternalReferee::receive_packet(const boost::system::error_code& error, size_t num_bytes) {
    if (error != boost::system::errc::success) {
        std::cerr << "Error receiving: " << error << " in " __FILE__ << std::endl;
        return;
    }

    SSL_Referee ref_packet;
    if (!ref_packet.ParseFromArray(recv_buffer_.data(), num_bytes)) {
        std::cerr << "NewRefereeModule: got bad packet of " << num_bytes << " bytes from "
                  << sender_endpoint_ << std::endl;
        std::cerr << "Address: " << &kRefereeAddress << std::endl;
        return;
    }

    // Publish the raw packet
    RawProtobufMsg msg;
    msg.data.resize(ref_packet.ByteSizeLong());
    if (!ref_packet.SerializeToArray(msg.data.data(), msg.data.size())) {
        EZ_ERROR("Failed to serialize referee packet.");
    }
    raw_ref_pub_->publish(msg);

    // Update and publish team information
    // FIXME(Kyle): Expiry times are only valid when we stay in the Playing
    // state
    TeamInfo blue_info = TeamInfo::from_refbox_packet(ref_packet.blue());
    TeamInfo yellow_info = TeamInfo::from_refbox_packet(ref_packet.yellow());
    set_team_info(blue_info, yellow_info);

    // Update game state
    handle_command(ref_packet.command());
    handle_stage(ref_packet.stage());
    set_stage_time_left(std::chrono::duration_cast<RJ::Seconds>(
        std::chrono::microseconds(ref_packet.stage_time_left())));
    send();
}

void ExternalReferee::setup_referee_multicast() {
    const auto any_address = boost::asio::ip::address_v4::any();
    boost::asio::ip::udp::endpoint listen_endpoint{any_address, kProtobufRefereePort};

    asio_socket_.open(listen_endpoint.protocol());
    asio_socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    try {
        asio_socket_.bind(listen_endpoint);
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Failed to bind to shared referee port");
    }
    // Join multicast group
    const boost::asio::ip::address multicast_address =
        boost::asio::ip::address::from_string(kRefereeAddress);
    asio_socket_.set_option(boost::asio::ip::multicast::join_group(multicast_address));
}

void ExternalReferee::update() {
    io_service_.poll();
    BallState state;
    spin_kick_detector(state);
}

void ExternalReferee::handle_command(SSL_Referee::Command command) {
    switch (command) {
        case SSL_Referee::HALT:
            halt();
            break;
        case SSL_Referee::STOP:
            stop();
            break;
        case SSL_Referee::NORMAL_START:
            break;
        case SSL_Referee::FORCE_START:
            play();
            break;
        case SSL_Referee::PREPARE_KICKOFF_YELLOW:
            restart(GameState::Restart::Kickoff, false);
            setup();
            break;
        case SSL_Referee::PREPARE_KICKOFF_BLUE:
            restart(GameState::Restart::Kickoff, true);
            setup();
            break;
        case SSL_Referee::PREPARE_PENALTY_YELLOW:
            restart(GameState::Restart::Penalty, false);
            setup();
            break;
        case SSL_Referee::PREPARE_PENALTY_BLUE:
            restart(GameState::Restart::Penalty, true);
            setup();
            break;
        case SSL_Referee::DIRECT_FREE_YELLOW:
            restart(GameState::Restart::Direct, false);
            play();
            break;
        case SSL_Referee::DIRECT_FREE_BLUE:
            restart(GameState::Restart::Direct, true);
            play();
            break;
        case SSL_Referee::INDIRECT_FREE_YELLOW:
            restart(GameState::Restart::Indirect, false);
            play();
            break;
        case SSL_Referee::INDIRECT_FREE_BLUE:
            restart(GameState::Restart::Indirect, true);
            play();
            break;
        case SSL_Referee::TIMEOUT_YELLOW:
        case SSL_Referee::TIMEOUT_BLUE:
            halt();
            break;
        case SSL_Referee::GOAL_YELLOW:
        case SSL_Referee::GOAL_BLUE:
            break;
        case SSL_Referee::BALL_PLACEMENT_YELLOW:
            // TODO(#1559): ball placement point
            ball_placement(rj_geometry::Point{}, false);
            stop();
            break;
        case SSL_Referee::BALL_PLACEMENT_BLUE:
            // TODO(#1559): ball placement point
            ball_placement(rj_geometry::Point{}, true);
            stop();
            break;
    }
}

GameState::Period ExternalReferee::period_from_proto(SSL_Referee::Stage stage) {
    switch (stage) {
        case SSL_Referee::NORMAL_FIRST_HALF_PRE:
        case SSL_Referee::NORMAL_FIRST_HALF:
            return GameState::FirstHalf;
        case SSL_Referee::NORMAL_HALF_TIME:
            return GameState::Halftime;
        case SSL_Referee::NORMAL_SECOND_HALF_PRE:
        case SSL_Referee::NORMAL_SECOND_HALF:
            return GameState::SecondHalf;
        case SSL_Referee::EXTRA_TIME_BREAK:
            return GameState::FirstHalf;
        case SSL_Referee::EXTRA_FIRST_HALF_PRE:
        case SSL_Referee::EXTRA_FIRST_HALF:
            return GameState::Overtime1;
        case SSL_Referee::EXTRA_HALF_TIME:
            return GameState::Halftime;
        case SSL_Referee::EXTRA_SECOND_HALF_PRE:
        case SSL_Referee::EXTRA_SECOND_HALF:
            return GameState::Overtime2;
        case SSL_Referee::PENALTY_SHOOTOUT_BREAK:
        case SSL_Referee::PENALTY_SHOOTOUT:
            return GameState::PenaltyShootout;
        case SSL_Referee::POST_GAME:
            return GameState::Overtime2;
        default:
            return GameState::FirstHalf;
    }
}

void ExternalReferee::handle_stage(SSL_Referee::Stage stage) {
    set_period(ExternalReferee::period_from_proto(stage));
}

}  // namespace referee
