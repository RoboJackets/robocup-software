#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio.hpp>
#include <boost/config.hpp>
#include <fmt/ostream.h>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_game_common/play_state.hpp>
#include <rj_geometry/ssl_ball_constants.hpp>
#include <rj_motion_common/world_state.hpp>
#include <rj_protos/state/ssl_gc_referee_message.pb.h>
#include <rj_ros_common/topic_names.hpp>
#include <rj_vision_msgs/msg/raw_protobuf.hpp>
#include <unistd.h>

#include "rj_referee/match_state.hpp"
#include "rj_referee/referee_base.hpp"
#include "rj_referee/team_info.hpp"

namespace referee {

/*
 * The multicast address we expect referee packets to come from, given by the league.
 * ExternalReferee joins this multicast group and listens for packets from it.
 */
static const std::string kRefereeSourceAddress = "224.5.23.1";
static const int kProtobufRefereePort = 10003;

using RawProtobufMsg = rj_vision_msgs::msg::RawProtobuf;
using Stage = Referee_Stage;
using Command = Referee_Command;
/**
 * @brief The ExternalReferee listens to a port for referee packets over the
 * network.
 *
 * @details Referee packets are sent out from the
 * [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller).
 * Referee packets contains info about which stage of the game it is, team
 * scores, yellow/red cards, etc.
 */
class ExternalReferee : public RefereeBase {
public:
    explicit ExternalReferee();

private:
    /**
     * Listen on the network and publish results.
     */
    void update();

    static MatchState::Period period_from_proto(Referee::Stage stage);

    rclcpp::Publisher<RawProtobufMsg>::SharedPtr raw_ref_pub_;
    rclcpp::TimerBase::SharedPtr network_timer_;

    using Command = std::pair<Referee::Command, std::optional<rj_geometry::Point>>;

    // Process a new referee command
    void handle_command(const Command& command);
    void handle_stage(Referee::Stage stage);

    // Arbitrary receive buffer size
    static constexpr size_t kRecvBufferSize = std::numeric_limits<uint16_t>::max() + 1;
    std::array<char, kRecvBufferSize> recv_buffer_{};

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket asio_socket_;
    boost::asio::ip::udp::endpoint sender_endpoint_;

    void setup_referee_multicast();
    void start_receive();
    void receive_packet(const boost::system::error_code& error, size_t num_bytes);

    std::optional<Command> last_command_;

    std::string param_team_name_;
};

void spin_external_referee();

}  // namespace referee
