#pragma once

#include <QUdpSocket>

#include "Radio.hpp"

class SimRadio: public Radio
{
public:
	SimRadio();
	
	virtual bool isOpen() const;
	virtual void send(const Packet::RadioTx &packet);
	virtual void receive();
	
private:
	QUdpSocket _socket;
	int _channel;
};
