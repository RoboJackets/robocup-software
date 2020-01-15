#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>
#include "radio/NetworkRadio.hpp"
#include "radio/PacketConvert.hpp"
#include "soccer/Processor.hpp"
#include "radio/SimRadio.hpp"
#include "Radio.hpp"

Radio::Radio(Context* context, bool sim, bool blueTeam)
    : _context(context){

    _simulation = sim;
    _blueTeam = blueTeam;
    _multipleManual = multipleManual;
    _manualID = manualID;

    // Create radio socket
    _radio =
            _simulation
            ? static_cast<Radio*>(new SimRadio(&_context, _blueTeam))
            : static_cast<Radio*>(new NetworkRadio(NetworkRadioServerPort));
}

void Radio::run(){
    // Read radio reverse packets
    _radio->receive();

    while (_radio->hasReversePackets()) {
        Packet::RadioRx rx = _radio->popReversePacket();
        _context.state.logFrame->add_radio_rx()->CopyFrom(rx);

        _context.lastRadioRxTime =
                RJ::Time(chrono::microseconds(rx.timestamp()));

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

    // Send motion commands to the robots
    if (_radio) {
        construct_tx_proto((*_context.state.logFrame->mutable_radio_tx()),
                           _context.robot_intents, _context.motion_setpoints);
        _radio->send(*_context.state.logFrame->mutable_radio_tx());
    }
}