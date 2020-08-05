#include "GrSimCommunicator.hpp"

#include <rj_common/Network.hpp>
#include <rj_constants/constants.hpp>

using namespace boost::asio;
using namespace Packet;

GrSimCommunicator::GrSimCommunicator(Context* context)
    : _context(context), _asio_socket(_io_service) {
    _asio_socket.open(ip::udp::v4());
    _grsim_endpoint = ip::udp::endpoint(ip::udp::v4(), SimCommandPort);
}

void GrSimCommunicator::run() {
    if (_context->grsim_command) {
        sendSimCommand(_context->grsim_command.value());
        _context->grsim_command = std::nullopt;
    }

    if (_context->ball_command && _context->screen_to_world_command) {
        placeBall(_context->ball_command.value(),
                  _context->screen_to_world_command.value());
        _context->ball_command = std::nullopt;
        _context->screen_to_world_command = std::nullopt;
    }
}

void GrSimCommunicator::placeBall(QPointF pos,
                                  Geometry2d::TransformMatrix screen_to_world) {
    grSim_Packet sim_packet;
    grSim_BallReplacement* ball_replace =
        sim_packet.mutable_replacement()->mutable_ball();

    ball_replace->set_x((screen_to_world * pos).x());
    ball_replace->set_y((screen_to_world * pos).y());
    ball_replace->set_vx(0);
    ball_replace->set_vy(0);

    sendSimCommand(sim_packet);
}

void GrSimCommunicator::sendSimCommand(const grSim_Packet& cmd) {
    std::string out;
    cmd.SerializeToString(&out);
    size_t bytes =
        _asio_socket.send_to(boost::asio::buffer(out), _grsim_endpoint);
    if (bytes == 0) {
        std::cerr << "Sent 0 bytes in " __FILE__ << std::endl;
    }
}
