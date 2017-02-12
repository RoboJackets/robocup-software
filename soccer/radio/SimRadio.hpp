#pragma once

#include <QUdpSocket>
#include <SystemState.hpp>

#include "Radio.hpp"

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    SimRadio(SystemState& system_state, bool blueTeam = false);

    virtual bool isOpen() const override;
    virtual void send(Packet::RadioTx& packet) override;
    virtual void receive() override;
    virtual void switchTeam(bool blueTeam) override;

private:
    SystemState& _state;;

    QUdpSocket _socket;
    int _channel;
    bool blueTeam;
};
