#pragma once

#include <mutex>

#include <boost/config.hpp>

#include <boost/asio.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>

#include "Radio.hpp"

#include "rc-fshare/rtp.hpp"

/**
 * @brief Interface for the radio over regular network interface
 */
class NetworkRadio : public Radio {
public:
    NetworkRadio(int server_port, int robot_port);
    ~NetworkRadio();

    virtual bool isOpen() const override;

    // This `send` method actually sends separate control packets to all robots
    // to maintain backwards compatibility with the old radio API.
    // TODO(Kyle) Rearchitect radio code to avoid needing to do this.
    virtual void send(Packet::RadioTx& packet) override;

    virtual void receive() override;

    virtual void switchTeam(bool) override;

protected:
    // Bidrectional mapping from IP address <-> robot ID
    using RobotIpMap = boost::bimaps::bimap<
        boost::bimaps::set_of<int>, boost::bimaps::multiset_of<boost::asio::ip::address>>;
    RobotIpMap _robot_ip_map;

    bool open();
    void receivePacket(const boost::system::error_code& error, std::size_t num_bytes);

    void startReceive();

    void registerRobot(int robot, boost::asio::ip::address ip);

    boost::asio::io_service _context;
    boost::asio::ip::udp::socket _socket;

    // Written by `async_receive_from`.
    std::array<char, rtp::ReverseSize> _recv_buffer;
    boost::asio::ip::udp::endpoint _robot_endpoint;

    int _server_port, _robot_port;
};
