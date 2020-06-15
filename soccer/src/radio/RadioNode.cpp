#include <gameplay/GameplayModule.hpp>

#include <rj_protos/LogFrame.pb.h>
#include <rj_protos/RadioRx.pb.h>
#include <rj_protos/RadioTx.pb.h>
#include <rj_protos/messages_robocup_ssl_detection.pb.h>
#include <rj_protos/messages_robocup_ssl_geometry.pb.h>
#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>

#include <Robot.hpp>
#include <rj_common/Network.hpp>
#include "NetworkRadio.hpp"
#include "PacketConvert.hpp"
#include "Radio.hpp"
#include "RadioNode.hpp"
#include "SimRadio.hpp"

RadioNode::RadioNode(Context* context, bool simulation, bool blueTeam)
    : _context(context) {
    _lastRadioRxTime = RJ::Time(std::chrono::microseconds(RJ::timestamp()));
    _simulation = simulation;
    _was_blue_team = blueTeam;
    _radio =
        _simulation
            ? static_cast<Radio*>(new SimRadio(_context, blueTeam))
            : static_cast<Radio*>(new NetworkRadio(NetworkRadioServerPort));
}

bool RadioNode::isOpen() { return _radio->isOpen(); }

RJ::Time RadioNode::getLastRadioRxTime() { return _lastRadioRxTime; }

Radio* RadioNode::getRadio() { return _radio; }

void RadioNode::switchTeam(bool blueTeam) { _radio->switchTeam(blueTeam); }

void RadioNode::run() {
    if (_context->game_state.blueTeam != _was_blue_team) {
        _was_blue_team = _context->game_state.blueTeam;
        _radio->switchTeam(_was_blue_team);
    }

    // Read radio reverse packets
    _radio->receive();

    while (_radio->hasReversePackets()) {
        RobotStatus rx = _radio->popReversePacket();

        _lastRadioRxTime = rx.timestamp;

        // Store this packet in the appropriate robot
        if (rx.shell_id < Num_Shells) {
            _context->robot_status[rx.shell_id] = rx;
            _context->state.self[rx.shell_id]->radioRxUpdated();
        }
    }

    _radio->send(_context->robot_intents, _context->motion_setpoints);
}
