#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_protos/LogFrame.pb.h>
#include <rj_protos/referee.pb.h>
#include <rj_msgs/msg/raw_protobuf.hpp>

#include "RefereeBase.hpp"
#include "config_client/config_client.h"

#include <boost/asio.hpp>
#include <boost/config.hpp>
#include <cstdint>
#include <mutex>
#include <rj_common/RefereeEnums.hpp>
#include <rj_common/Utils.hpp>
#include <thread>
#include <vector>

#include "GameState.hpp"
#include "Node.hpp"
#include "SystemState.hpp"
#include "TeamInfo.hpp"

namespace referee {

using RawProtobufMsg = rj_msgs::msg::RawProtobuf;

/**
 * @brief The ref module listens to a port for referee packets over the network.
 *
 * @details ExternalReferee packets are sent out from the
 * [ssl-refbox](https://github.com/Hawk777/ssl-refbox) program.
 * You can see the [protobuf
 * packet](https://github.com/Hawk777/ssl-refbox/blob/master/referee.proto) for
 * full details,
 * but the packets contains info about which stage of the game it is, team
 * scores, yellow/red cards, etc.
 *
 * Each time a new packet arrives, the ref module updates the GameState object
 * with the new information.
 */
class ExternalReferee : public RefereeBase {
public:
    explicit ExternalReferee();

    void run();

protected:
    static GameState::Period period_from_proto(SSL_Referee::Stage stage);

    rclcpp::Publisher<RawProtobufMsg>::SharedPtr _raw_ref_pub;

    // The number of commands issued since startup
    int64_t _command_counter = 0;

    // The UNIX timestamp when the command was issued, in microseconds.
    // This value changes only when a new command is issued, not on each packet.
    RJ::Time _last_command_time{};

    // Process a new referee command
    void handle_command(SSL_Referee::Command command);
    void handle_stage(SSL_Referee::Stage stage);

    Geometry2d::Point _readyBallPos;

private:
    // Arbitrary receive buffer size
    static constexpr size_t RecvBufferSize =
        std::numeric_limits<uint16_t>::max() + 1;
    std::array<char, RecvBufferSize> _recv_buffer;

    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _asio_socket;
    boost::asio::ip::udp::endpoint _sender_endpoint;

    void setupRefereeMulticast();
    void startReceive();
    void receivePacket(const boost::system::error_code& error,
                       size_t num_bytes);
};

void spin_external_referee();

} // namespace referee
