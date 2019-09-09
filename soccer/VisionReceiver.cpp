#include "VisionReceiver.hpp"

#include <unistd.h>
#include <QMutexLocker>
#include <QUdpSocket>
#include <Utils.hpp>
#include <multicast.hpp>
#include <stdexcept>

using namespace std;
using boost::asio::ip::udp;

VisionReceiver::VisionReceiver(Context* context, bool sim, int port)
    : port(port), _context(context), _socket(_io_context) {
    _recv_buffer.resize(65536);

    // There should be at most four packets: one for each camera on each of two
    // frames, assuming some clock skew between this
    // computer and the vision computer.
    _packets.reserve(4);

    setPort(port);
}

void VisionReceiver::setPort(int port) {
    // If the socket is already open, close it to cancel any pending operations
    // before we reopen it on a new port.
    if (_socket.is_open()) {
        _socket.close();
    }

    // Open the socket and allow address reuse. We need this to allow multiple
    // instances of soccer to listen on the same multicast port.
    _socket.open(udp::v4());
    _socket.set_option(udp::socket::reuse_address(true));

    // Set up multicast.
    if (!multicast_add_native(_socket.native_handle(), SharedVisionAddress)) {
        std::cerr << "Multicast add failed" << std::endl;
        return;
    }

    // Bind the socket.
    boost::system::error_code bind_error;
    _socket.bind(udp::endpoint(udp::v4(), port), bind_error);
    if (bind_error) {
        std::cerr << "Vision port bind failed with error: "
                  << bind_error.message() << std::endl;
        return;
    }

    // Start receiving. Note that this only listens once; any later receive
    // will call `startReceive` again to continue listening.
    startReceive();
}

void VisionReceiver::run() {
    // Let boost::asio check for new packets
    _io_context.poll();

    // Move new packets into context using a move_iterator.
    _context->vision_packets.insert(_context->vision_packets.begin(),
                                    std::make_move_iterator(_packets.begin()),
                                    std::make_move_iterator(_packets.end()));

    // Remove the nullptr packets that we just moved to context.
    _packets.clear();
}

void VisionReceiver::startReceive() {
    // Set a receive callback
    _socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _sender_endpoint,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            // Handle the packet and then restart reception to allow new
            // incoming data
            receivePacket(error, num_bytes);
            startReceive();
        });
}

void VisionReceiver::receivePacket(const boost::system::error_code& error,
                                   std::size_t num_bytes) {
    // Check for error
    if (error) {
        std::cerr << "Vision receive failed with error: " << error.message()
                  << std::endl;
        return;
    }

    static int num = 0;

    // Parse the protobuf message
    std::unique_ptr<VisionPacket> packet = std::make_unique<VisionPacket>();
    packet->receivedTime = RJ::now();
    if (!packet->wrapper.ParseFromArray(&_recv_buffer[0], num_bytes)) {
        std::cerr << "VisionReceiver: got bad packet of " << num_bytes
                  << " bytes from " << _sender_endpoint.address() << ":"
                  << _sender_endpoint.port() << std::endl;
        return;
    }

    // Put the message into the list
    _packets.push_back(std::move(packet));
}
