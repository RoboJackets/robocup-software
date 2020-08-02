#pragma once

#include <rj_protos/LogFrame.pb.h>
#include <rj_protos/referee.pb.h>

#include <boost/asio.hpp>
#include <boost/config.hpp>
#include <cstdint>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rj_common/RefereeEnums.hpp>
#include <rj_common/Utils.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <thread>
#include <vector>

#include "GameState.hpp"
#include "Node.hpp"
#include "RefereeBase.hpp"
#include "SystemState.hpp"
#include "TeamInfo.hpp"
#include "config_client/config_client.h"

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

    static GameState::Period period_from_proto(SSL_Referee::Stage stage);

    rclcpp::Publisher<RawProtobufMsg>::SharedPtr _raw_ref_pub;
    rclcpp::TimerBase::SharedPtr _network_timer;

    // Process a new referee command
    void handle_command(SSL_Referee::Command command);
    void handle_stage(SSL_Referee::Stage stage);

    // Arbitrary receive buffer size
    static constexpr size_t RecvBufferSize =
        std::numeric_limits<uint16_t>::max() + 1;
    std::array<char, RecvBufferSize> _recv_buffer{};

    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _asio_socket;
    boost::asio::ip::udp::endpoint _sender_endpoint;

    void setupRefereeMulticast();
    void startReceive();
    void receivePacket(const boost::system::error_code& error,
                       size_t num_bytes);
};

void spin_external_referee();

}  // namespace referee
