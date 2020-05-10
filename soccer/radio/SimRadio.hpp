#pragma once

#include <Context.hpp>
#include <SystemState.hpp>
#include <boost/asio.hpp>
#include <cstdint>

#include "Radio.hpp"

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    static std::size_t instance_count;
    SimRadio(Context* const context, bool blueTeam = false);

    bool isOpen() const override;
    void send(Packet::RadioTx& radioTx) override;
    void receive() override;
    void switchTeam(bool blueTeam) override;

    void stopRobots();

private:
    Context* const _context;

    void handleReceive(uint8_t data);
    void startReceive();
    void receivePacket(const boost::system::error_code& error,
                       size_t num_bytes);

    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _grsim_endpoint;

    std::vector<char> _buffer;

    bool _blueTeam;
};
