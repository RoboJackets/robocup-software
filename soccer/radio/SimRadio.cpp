#include "SimRadio.hpp"

#include <Network.hpp>
#include <stdexcept>

using namespace std;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

SimRadio::SimRadio()
{
	// No channel specified.
	// Pick the first available one.
	if (_socket.bind(RadioRxPort))
	{
		_channel = 0;
	} else {
		if (_socket.bind(RadioRxPort + 1))
		{
			_channel = 1;
		} else {
			throw runtime_error("Can't bind to either radio port");
		}
	}
}

bool SimRadio::isOpen() const
{
	//FIXME - check the socket
	return true;
}

void SimRadio::send(const Packet::RadioTx& packet)
{
	std::string out;
	packet.SerializeToString(&out);
	_socket.writeDatagram(&out[0], out.size(), LocalAddress, RadioTxPort + _channel);
}

void SimRadio::receive()
{
	while (_socket.hasPendingDatagrams())
	{
		unsigned int n = _socket.pendingDatagramSize();
		string buf;
		buf.resize(n);
		_socket.readDatagram(&buf[0], n);
		
		_reversePackets.push_back(RadioRx());
		RadioRx &packet = _reversePackets.back();
		
		if (!packet.ParseFromString(buf))
		{
			printf("Bad radio packet of %d bytes\n", n);
			continue;
		}
	}
}
