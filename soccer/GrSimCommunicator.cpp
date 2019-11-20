#include "GrSimCommunicator.hpp"

#include <Constants.hpp>
#include <Network.hpp>

using namespace boost;
using namespace Packet;

GrSimCommunicator::GrSimCommunicator(Context* context) : _context(context) {}

void GrSimCommunicator::run() {
    if (_context->grsim_command) {
        sendSimCommand(_context->grsim_command.value());
        _context->grsim_command = std::nullopt;
    }

    if (_context->ball_command && _context->screen_to_world) {
        placeBall(_context->ball_command.value(), _context->screen_to_world.value());
        _context->ball_command = std::nullopt;
        _context->screen_to_world = std::nullopt;
    }
}


void GrSimCommunicator::placeBall(QPointF pos,
                                  Geometry2d::TransformMatrix _screenToWorld) {
    grSim_Packet simPacket;
    grSim_BallReplacement* ball_replace =
        simPacket.mutable_replacement()->mutable_ball();

    ball_replace->mutable_pos()->set_x((_screenToWorld * pos).x());
    ball_replace->mutable_pos()->set_y((_screenToWorld * pos).y());
    ball_replace->mutable_vel()->set_x(0);
    ball_replace->mutable_vel()->set_y(0);

    sendSimCommand(simPacket);
}

void GrSimCommunicator::sendSimCommand(const grSim_Packet& cmd) {
    std::string out;
    cmd.SerializeToString(&out);
    _simCommandSocket.writeDatagram(&out[0], out.size(),
                                    QHostAddress(QHostAddress::LocalHost),
                                    SimCommandPort);
}
