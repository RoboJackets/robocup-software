#pragma once

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>

class Radio
{
public:
	virtual bool isOpen() const = 0;
	virtual void send(const Packet::RadioTx &packet) = 0;
	virtual void receive() = 0;

	const std::vector<Packet::RadioRx> &reversePackets() const
	{
		return _reversePackets;
	}
	
	void clear()
	{
		_reversePackets.clear();
	}

protected:
	std::vector<Packet::RadioRx> _reversePackets;
};
