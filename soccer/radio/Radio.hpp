#pragma once

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>

class Radio
{
public:
	Radio()
	{
		_channel = 0;
    }
	
	virtual bool isOpen() const = 0;
	virtual void send(Packet::RadioTx &packet) = 0;
    virtual void receive() = 0;

    virtual void switchTeam(bool blueTeam) = 0;
	
	virtual void channel(int n)
	{
		_channel = n;
	}
	
	int channel() const
	{
		return _channel;
	}

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
	int _channel;
};
