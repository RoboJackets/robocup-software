#include "SimRadio.hpp"

#include <Network.hpp>
#include <stdexcept>

using namespace std;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

SimRadio::SimRadio(bool blueTeam) {
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
}

bool SimRadio::isOpen() const {
    // FIXME - check the socket
    return true;
}

void SimRadio::send(Packet::RadioTx& packet) {
    std::string out;
    packet.SerializeToString(&out);
    _socket.writeDatagram(&out[0], out.size(), LocalAddress,
                          RadioTxPort + _channel);
}

void SimRadio::receive() {
    while (_socket.hasPendingDatagrams()) {
        unsigned int n = _socket.pendingDatagramSize();
        string buf;
        buf.resize(n);
        _socket.readDatagram(&buf[0], n);

        _reversePackets.push_back(RadioRx());
        RadioRx& packet = _reversePackets.back();

        if (!packet.ParseFromString(buf)) {
            printf("Bad radio packet of %d bytes\n", n);
            continue;
        }
    }
}

void SimRadio::switchTeam(bool blueTeam) {
    _socket.close();
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
}
