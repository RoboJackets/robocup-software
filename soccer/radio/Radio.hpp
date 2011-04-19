#pragma once

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>

class Radio
{
public:
	virtual bool isOpen() const = 0;
	virtual void send(const Packet::RadioTx &packet) = 0;
	virtual bool receive(Packet::RadioRx *packet) = 0;
};
