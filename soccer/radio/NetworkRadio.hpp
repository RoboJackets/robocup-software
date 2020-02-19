#pragma once

#include <mutex>

#include <boost/config.hpp>

#include <RobotIntent.hpp>
#include <boost/asio.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>

#include "Radio.hpp"

#include "rc-fshare/rtp.hpp"

/**
 * @brief Interface for the radio over regular network interface
 *
 * TODO(Kyle): Clean this up by removing dual-radio support.
 */
class NetworkRadio : public Radio {
public:
    NetworkRadio(int server_port);

    virtual bool isOpen() const override;

    // This `send` method actually sends separate control packets to all robots
    // to maintain backwards compatibility with the old radio API.
    // TODO(Kyle) Rearchitect radio code to avoid needing to do this.
    virtual void send(Packet::RadioTx& radioTx) override;

    virtual void receive() override;

    virtual void switchTeam(bool) override;

protected:
    struct RobotConnection {
        boost::asio::ip::udp::endpoint endpoint;
        RJ::Time last_received;
    };

    // Connections to the robots, indexed by robot ID.
    std::vector<std::optional<RobotConnection>> _connections;

    // Map from IP address to robot ID.
    std::map<boost::asio::ip::udp::endpoint, int> _robot_ip_map;

    bool open();
    void receivePacket(const boost::system::error_code& error,
                       std::size_t num_bytes);

    void startReceive();

    boost::asio::io_service _context;
    boost::asio::ip::udp::socket _socket;

    // Written by `async_receive_from`.
    std::array<char, rtp::ReverseSize> _recv_buffer;
    boost::asio::ip::udp::endpoint _robot_endpoint;

    // Read from by `async_send_to`
    std::vector<
        std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>>
        _send_buffers;

    constexpr static std::chrono::duration kTimeout = std::chrono::milliseconds(250);
};
