#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>
#include "radio/NetworkRadio.hpp"
#include "radio/PacketConvert.hpp"
#include "soccer/Processor.hpp"
#include "radio/SimRadio.hpp"
#include "Radio.hpp"

Radio::Radio(Context* context, bool sim, bool blueTeam, bool multipleManual, int manualID, std::vector<Joystick*> joys)
    : _context(context){

    _simulation = sim;
    _blueTeam = blueTeam;
    _multipleManual = multipleManual;
    _manualID = manualID;
    _joysticks = joys;

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
    sendRadioData();
}

void Radio::sendRadioData() {
    // Halt overrides normal motion control, but not joystick
    if (_context.game_state.halt()) {
        // Force all motor speeds to zero
        for (OurRobot* r : _context.state.self) {
            RobotIntent& intent = _context.robot_intents[r->shell()];
            MotionSetpoint& setpoint = _context.motion_setpoints[r->shell()];
            setpoint.xvelocity = 0;
            setpoint.yvelocity = 0;
            setpoint.avelocity = 0;
            intent.dvelocity = 0;
            intent.kcstrength = 0;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
            intent.song = RobotIntent::Song::STOP;
        }
    }

    // Add RadioTx commands for visible robots and apply joystick input
    std::vector<int> manualIds = getJoystickRobotIds();
    for (OurRobot* r : _context.state.self) {
        RobotIntent& intent = _context.robot_intents[r->shell()];
        if (r->visible() || _manualID == r->shell() || _multipleManual) {
            intent.is_active = true;
            // MANUAL STUFF
            if (_multipleManual) {
                auto info =
                        find(manualIds.begin(), manualIds.end(), r->shell());
                int index = info - manualIds.begin();

                // figure out if this shell value has been assigned to a
                // joystick
                // do stuff with that information such as assign it to the first
                // available
                if (info == manualIds.end()) {
                    for (int i = 0; i < manualIds.size(); i++) {
                        if (manualIds[i] == -1) {
                            index = i;
                            _joysticks[i]->setRobotId(r->shell());
                            manualIds[i] = r->shell();
                            break;
                        }
                    }
                }

                if (index < manualIds.size()) {
                    applyJoystickControls(
                            getJoystickControlValue(*_joysticks[index]), r);
                }
            } else if (_manualID == r->shell()) {
                auto controlValues = getJoystickControlValues();
                if (controlValues.size()) {
                    applyJoystickControls(controlValues[0], r);
                }
            }
        } else {
            intent.is_active = false;
        }
    }

    if (_radio) {
        construct_tx_proto((*_context.state.logFrame->mutable_radio_tx()),
                           _context.robot_intents, _context.motion_setpoints);
        _radio->send(*_context.state.logFrame->mutable_radio_tx());
    }
}

vector<int> Radio::getJoystickRobotIds() {
    vector<int> robotIds;
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) {
            robotIds.push_back(joy->getRobotId());
        } else {
            robotIds.push_back(-2);
        }
    }
    return robotIds;
}