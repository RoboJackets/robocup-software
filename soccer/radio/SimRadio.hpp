#pragma once

#include <QUdpSocket>

#include "Radio.hpp"

/**
 * @brief Radio IO with robots in the grSimulator
 */
class SimRadio : public Radio {
public:
    SimRadio(bool blueTeam = false);

    virtual bool isOpen() const override;
    virtual void send(Packet::RadioTx& packet) override;
    virtual void receive() override;
    virtual void switchTeam(bool blueTeam) override;

private:
    QUdpSocket _socket;
    int _channel;
};
