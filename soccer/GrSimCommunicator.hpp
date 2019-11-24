// An extension of FieldView that generates SimCommands in response
// to clicks/drags when live.

#pragma once

#include <protobuf/grSim_Commands.pb.h>
#include <protobuf/grSim_Packet.pb.h>
#include <protobuf/grSim_Replacement.pb.h>

#include <Geometry2d/TransformMatrix.hpp>
#include <QUdpSocket>

#include <Context.hpp>
#include <Node.hpp>

class GrSimCommunicator : public Node {
public:
    explicit GrSimCommunicator(Context* context);

    void sendSimCommand(const grSim_Packet& cmd);

    // Places the ball at a position on the screen
    void placeBall(QPointF pos, Geometry2d::TransformMatrix _screenToWorld);

    void run() override;

private:
    Context* _context;
    QUdpSocket _simCommandSocket;
};
