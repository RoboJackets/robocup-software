#include "grSimCom.hpp"

#include <Constants.hpp>
#include <Network.hpp>

using namespace boost;
using namespace Packet;


void grSimCom::placeBall(QPointF pos, Geometry2d::TransformMatrix _screenToWorld) {
    grSim_Packet simPacket;
    grSim_BallReplacement* ball_replace =
        simPacket.mutable_replacement()->mutable_ball();

    ball_replace->mutable_pos()->set_x((_screenToWorld * pos).x());
    ball_replace->mutable_pos()->set_y((_screenToWorld * pos).y());
    ball_replace->mutable_vel()->set_x(0);
    ball_replace->mutable_vel()->set_y(0);

    sendSimCommand(simPacket);
}

void grSimCom::sendSimCommand(const grSim_Packet& cmd) {
    std::string out;
    std::cout<<"send sim command??"<<std::endl;
    cmd.SerializeToString(&out);
    _simCommandSocket.writeDatagram(&out[0], out.size(),
                                    QHostAddress(QHostAddress::LocalHost),
                                    SimCommandPort);
}
