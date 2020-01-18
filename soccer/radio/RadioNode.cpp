#include "Constants.hpp"
#include "NetworkRadio.hpp"
#include "Radio.hpp"
#include "RadioNode.hpp"
#include "SimRadio.hpp"

RadioNode::RadioNode(Context *context, bool simulation, bool blueTeam): _context(context) {
    _blueTeam = blueTeam;
    _lastRadioRxTime = RJ::Time(chrono::microseconds(RJ::timestamp()));
    _simulation = simulation;
    _radio = _simulation
             ? static_cast<Radio*>(new SimRadio(&_context, _blueTeam))
             : static_cast<Radio*>(new NetworkRadio(NetworkRadioServerPort));
}

bool RadioNode::isOpen() {
    return _radio->.isOpen();
}

RJ::Time RadioNode::getLastRadioRxTime(){
    return _lastRadioRxTime;
}

Radio* RadioNode::getRadio(){
    return _radio;
}

void RadioNode::switchTeam(bool blueTeam) {
    _radio.switchTeam(blueTeam);
}

void RadioNode::run(){

    // Read radio reverse packets
    _radio->receive();

    while (_radio->hasReversePackets()) {
        Packet::RadioRx rx = _radio->popReversePacket();
        _context.state.logFrame->add_radio_rx()->CopyFrom(rx);

        _lastRadioRxTime = RJ::Time(chrono::microseconds(rx.timestamp()));

        // Store this packet in the appropriate robot
        unsigned int board = rx.robot_id();
        if (board < Num_Shells) {
            // We have to copy because the RX packet will survive past this
            // frame but LogFrame will not (the RadioRx in LogFrame will be
            // reused).
            _context.state.self[board]->setRadioRx(rx);
            _context.state.self[board]->radioRxUpdated();
        }
    }

    if (_radio) {
        construct_tx_proto((*_context.state.logFrame->mutable_radio_tx()),
                           _context.robot_intents, _context.motion_setpoints);
        _radio->send(*_context.state.logFrame->mutable_radio_tx());
    }

}
