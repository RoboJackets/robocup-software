#pragma once

#include <rtos.h>
#include "CC1201.hpp"
#include "CommModule.hpp"
#include "RtosTimerHelper.hpp"

class RadioProtocol {
public:
    enum State {
        STOPPED,
        DISCONNECTED,
        CONNECTED,
    };

    /// After this number of milliseconds without receiving a packet from the
    /// base station, we are considered "disconnected"
    static const uint32_t TIMEOUT_INTERVAL = 2000;

    RadioProtocol(std::shared_ptr<CommModule> commModule, CC1201* radio,
                  uint8_t uid = rtp::INVALID_ROBOT_UID)
        : _commModule(commModule),
          _radio(radio),
          _uid(uid),
          _state(STOPPED),
          _replyTimer(this, &RadioProtocol::reply, osTimerOnce),
          _timeoutTimer(this, &RadioProtocol::_timeout, osTimerOnce) {
        ASSERT(commModule != nullptr);
        ASSERT(radio != nullptr);
        _radio->setAddress(rtp::ROBOT_ADDRESS);
    }

    ~RadioProtocol() { stop(); }

    /// Set robot unique id.  Also update address.
    void setUID(uint8_t uid) { _uid = uid; }

    /**
     * Callback that is called whenever a packet is received.  Set this in
     * order to handle parsing the packet and creating a response.  This
     * callback
     * should return a formatted reply buffer, which will be sent in the
     * appropriate reply slot.
     *
     * @param msg A pointer to the start of the message addressed to this robot
     * @return formatted reply buffer
     */
    std::function<std::vector<uint8_t>(const rtp::ControlMessage* msg)>
        rxCallback;

    void start() {
        _state = DISCONNECTED;

        _commModule->setRxHandler(this, &RadioProtocol::rxHandler,
                                  rtp::Port::CONTROL);
        _commModule->setTxHandler((CommLink*)global_radio,
                                  &CommLink::sendPacket, rtp::Port::CONTROL);

        LOG(INF1, "Radio protocol listening on port %d", rtp::Port::CONTROL);
    }

    void stop() {
        _commModule->close(rtp::Port::CONTROL);

        _replyTimer.stop();
        _state = STOPPED;

        LOG(INF1, "Radio protocol stopped");
    }

    State state() const { return _state; }

    void rxHandler(rtp::packet pkt) {
        // LOG(INIT, "got pkt!");
        // TODO: check packet size before parsing
        bool addressed = false;
        const rtp::ControlMessage* msg;
        size_t slot;
        for (slot = 0; slot < 6; slot++) {
            size_t offset = slot * sizeof(rtp::ControlMessage);
            msg = (const rtp::ControlMessage*)(pkt.payload.data() + offset);

            if (msg->uid == _uid) {
                addressed = true;
                break;
            }
        }

        /// time, in ms, for each reply slot
        // TODO(justin): double-check this
        const uint32_t SLOT_DELAY = 2;

        if (addressed) {
            _state = CONNECTED;

            // reset timeout whenever we receive a packet
            _timeoutTimer.stop();
            _timeoutTimer.start(TIMEOUT_INTERVAL);

            _replyTimer.start(1 + SLOT_DELAY * slot);

            if (rxCallback) {
                _reply = std::move(rxCallback(msg));
            } else {
                LOG(WARN, "no callback set");
            }
        } else {
            // TODO(justin): reply in an available slot
        }
    }

private:
    void reply() {
        rtp::packet pkt;
        pkt.header.port = rtp::Port::CONTROL;
        pkt.header.type = rtp::header_data::Control;
        pkt.header.address = rtp::BASE_STATION_ADDRESS;

        pkt.payload = std::move(_reply);

        _commModule->send(std::move(pkt));
    }

    void _timeout() { _state = DISCONNECTED; }

    std::shared_ptr<CommModule> _commModule;
    CC1201* _radio;

    uint32_t _lastReceiveTime = 0;

    uint8_t _uid;
    State _state;

    std::vector<uint8_t> _reply;

    RtosTimerHelper _replyTimer;
    RtosTimerHelper _timeoutTimer;
};
