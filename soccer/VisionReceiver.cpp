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
    : simulation(sim), port(port), _context(context), _socket(_io_context) {
    _recv_buffer.resize(65536);

    // There should be at most four packets: one for each camera on each of two
    // frames, assuming some clock skew between this
    // computer and the vision computer.
    _packets.reserve(4);

    // TODO(Kyle): Make sure we don't need to use SimVisionPort
    boost::system::error_code bind_error;
    _socket.open(udp::v4());
    _socket.set_option(udp::socket::reuse_address(true));
    _socket.bind(udp::endpoint(udp::v4(), port), bind_error);
    if (!multicast_add_native(_socket.native_handle(), SharedVisionAddress)) {
        std::cerr << "Multicast add failed" << std::endl;
    }

    if (bind_error) {
        std::cerr << "Vision port bind failed with error: "
                  << bind_error.message() << std::endl;
    }

    startReceive();
}

void VisionReceiver::run() {
    // Let boost::asio check for new packets
    _io_context.poll();

    // Move new packets into Context
    _context->vision_packets.insert(_context->vision_packets.begin(),
                                    std::make_move_iterator(_packets.begin()),
                                    std::make_move_iterator(_packets.end()));
    _packets.clear();
}

void VisionReceiver::startReceive() {
    // Set a receive callback
    _socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _sender_endpoint,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receivePacket(error, num_bytes);
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

    startReceive();
}
