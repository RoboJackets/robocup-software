#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/config.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rj_common/referee_enums.hpp>
#include <rj_common/utils.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <rj_protos/LogFrame.pb.h>
#include <rj_protos/referee.pb.h>

#include "game_state.hpp"
#include "node.hpp"
#include "referee_base.hpp"
#include "team_info.hpp"

namespace referee {

using RawProtobufMsg = rj_msgs::msg::RawProtobuf;

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
