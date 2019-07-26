#pragma once

#include <Context.hpp>
#include <QUdpSocket>
#include <SystemState.hpp>

#include "Radio.hpp"

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    static std::size_t instance_count;
    SimRadio(Context* const context, bool blueTeam = false);

    virtual bool isOpen() const override;
    virtual void send(Packet::RadioTx& packet) override;
    virtual void receive() override;
    virtual void switchTeam(bool blueTeam) override;

    void stopRobots();

private:
    Context* const _context;

    QUdpSocket _tx_socket;
    QUdpSocket _rx_socket;
    int _channel;
    bool _blueTeam;
};
