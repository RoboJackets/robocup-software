#include "radio_node.hpp"

#include <robot.hpp>
#include <gameplay/gameplay_module.hpp>
#include <rj_common/network.hpp>
#include <rj_protos/LogFrame.pb.h>
#include <rj_protos/RadioRx.pb.h>
#include <rj_protos/RadioTx.pb.h>
#include <rj_protos/messages_robocup_ssl_detection.pb.h>
#include <rj_protos/messages_robocup_ssl_geometry.pb.h>
#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>

#include "network_radio.hpp"
#include "packet_convert.hpp"
#include "radio.hpp"
#include "sim_radio.hpp"

RadioNode::RadioNode(Context* context, bool simulation, bool blue_team) : context_(context) {
    last_radio_rx_time_ = RJ::Time(std::chrono::microseconds(RJ::timestamp()));
    simulation_ = simulation;
    was_blue_team_ = blue_team;
    radio_ = simulation_ ? static_cast<Radio*>(new SimRadio(context_, blue_team))
                         : static_cast<Radio*>(new NetworkRadio(kNetworkRadioServerPort));
}

bool RadioNode::is_open() { return radio_->is_open(); }

RJ::Time RadioNode::get_last_radio_rx_time() { return last_radio_rx_time_; }

Radio* RadioNode::get_radio() { return radio_; }

void RadioNode::switch_team(bool blue_team) { radio_->switch_team(blue_team); }

void RadioNode::run() {
    if (context_->blue_team != was_blue_team_) {
        was_blue_team_ = context_->blue_team;
        radio_->switch_team(was_blue_team_);
    }

    // Read radio reverse packets
    radio_->receive();

    while (radio_->has_reverse_packets()) {
        RobotStatus rx = radio_->pop_reverse_packet();

        last_radio_rx_time_ = rx.timestamp;

        // Store this packet in the appropriate robot
        if (rx.shell_id < kNumShells) {
            context_->robot_status[rx.shell_id] = rx;
            context_->state.self[rx.shell_id]->radio_rx_updated();
        }
    }

    radio_->send(context_->robot_intents, context_->motion_setpoints);
}
