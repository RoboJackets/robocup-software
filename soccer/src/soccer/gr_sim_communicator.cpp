#include "gr_sim_communicator.hpp"

#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_constants/constants.hpp>

using namespace boost::asio;
using namespace Packet;

GrSimCommunicator::GrSimCommunicator(Context* context)
    : context_(context), asio_socket_(io_service_) {
    asio_socket_.open(ip::udp::v4());
    grsim_endpoint_ = ip::udp::endpoint(ip::udp::v4(), kSimCommandPort);
}

void GrSimCommunicator::run() {
    if (context_->grsim_command) {
        send_sim_command(context_->grsim_command.value());
        context_->grsim_command = std::nullopt;
    }

    if (context_->ball_command && context_->screen_to_world_command) {
        place_ball(context_->ball_command.value(), context_->screen_to_world_command.value());
        context_->ball_command = std::nullopt;
        context_->screen_to_world_command = std::nullopt;
    }
}

void GrSimCommunicator::place_ball(QPointF pos, rj_geometry::TransformMatrix screen_to_world) {
    grSim_Packet sim_packet;
    grSim_BallReplacement* ball_replace = sim_packet.mutable_replacement()->mutable_ball();

    ball_replace->set_x((screen_to_world * pos).x());
    ball_replace->set_y((screen_to_world * pos).y());
    ball_replace->set_vx(0);
    ball_replace->set_vy(0);

    send_sim_command(sim_packet);
}

void GrSimCommunicator::send_sim_command(const grSim_Packet& cmd) {
    std::string out;
    cmd.SerializeToString(&out);
    size_t bytes = asio_socket_.send_to(boost::asio::buffer(out), grsim_endpoint_);
    if (bytes == 0) {
        SPDLOG_ERROR("Sent 0 bytes.");
    }
}
