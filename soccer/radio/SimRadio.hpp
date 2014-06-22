#pragma once

#include <QUdpSocket>

#include "Radio.hpp"

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio: public Radio
{
public:
    SimRadio(bool blueTeam = false);

	virtual bool isOpen() const;
	virtual void send(Packet::RadioTx &packet);
    virtual void receive();
    virtual void switchTeam(bool blueTeam);
	
private:
	QUdpSocket _socket;
	int _channel;
};
