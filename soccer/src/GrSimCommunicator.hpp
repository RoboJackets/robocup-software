#pragma once

#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/grSim_Replacement.pb.h>

#include <Context.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <Node.hpp>
#include <boost/asio.hpp>

class GrSimCommunicator : public Node {
public:
    explicit GrSimCommunicator(Context* context);

    void sendSimCommand(const grSim_Packet& cmd);

    // Places the ball at a position on the screen
    void placeBall(QPointF pos, Geometry2d::TransformMatrix _screenToWorld);

    void run() override;

private:
    Context* _context;
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _asio_socket;
    boost::asio::ip::udp::endpoint _grsim_endpoint;
};
