#pragma once

#include <rtos.h>
#include "CommModule.hpp"
#include "CC1201.hpp"

class RadioProtocol2011 {
public:
    enum State {
        STOPPED,
        DISCONNECTED,
        CONNECTED,
    };

    /// After this number of milliseconds without receiving a packet from the
    /// base station, we are considered "disconnected"
    static const uint32_t TIMEOUT_INTERVAL = 2000;

    RadioProtocol2011(std::shared_ptr<CommModule> commModule, CC1201* radio,
                      uint8_t uid = 0)
        : _commModule(commModule),
          _radio(radio),
          _uid(uid),
          _state(STOPPED),
          _replyTimer(&replyTimerFired, osTimerOnce, this),
          _timeoutTimer(&timeoutTimerFired, osTimerOnce, this) {
        ASSERT(commModule != nullptr);
        ASSERT(radio != nullptr);
    }

    ~RadioProtocol2011() {
        stop();
    }

    /// robot unique id
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
    std::function<std::vector<uint8_t>(uint8_t* msg)> rxCallback;

    void start() {
        _state = DISCONNECTED;

        _commModule->setRxHandler(this, &RadioProtocol2011::rxHandler,
                                  rtp::port::CONTROL);
        _commModule->setTxHandler((CommLink*)global_radio,
                                  &CommLink::sendPacket, rtp::port::CONTROL);

        LOG(INF1, "Radio protocol listening on port %d", rtp::port::CONTROL);
    }

    void stop() {
        _commModule->close(rtp::port::CONTROL);

        _replyTimer.stop();
        _state = STOPPED;

        LOG(INF1, "Radio protocol stopped");
    }

    State state() const { return _state; }

    void rxHandler(rtp::packet* pkt) {
        bool addressed = false;
        size_t offset, slot;
        for (slot = 0; slot < 6; slot++) {
            offset = 1 + slot * 9;  // slots are 9 bytes in size
            uint8_t shellId = pkt->payload[offset + 4] & 0x0f;

            if (shellId == _uid) {
                addressed = true;
                break;
            }
        }

        /// time, in ms, for each reply slot
        const uint32_t SLOT_DELAY = 2;

        if (addressed) {
            _state = CONNECTED;

            // reset timeout whenever we receive a packet
            _timeoutTimer.stop();
            _timeoutTimer.start(TIMEOUT_INTERVAL);

            _replyTimer.start(slot * SLOT_DELAY);

            if (rxCallback) {
                _reply = std::move(rxCallback(pkt->payload.data() + offset));
            }
        }
    }

private:
    static void replyTimerFired(const void* instance) {
        RadioProtocol2011* thiss = const_cast<RadioProtocol2011*>(
            reinterpret_cast<const RadioProtocol2011*>(instance));
        thiss->reply();
    }

    void reply() {
        rtp::packet pkt;
        pkt.header.port = rtp::port::CONTROL;
        pkt.header.type = rtp::header_data::Control;
        pkt.header.address = rtp::BASE_STATION_ADDRESS;

        pkt.payload = std::move(_reply);

        _commModule->send(pkt);
    }

    /// Transitition to DISCONNECTED if timeout timer fires
    static void timeoutTimerFired(const void* instance) {
        RadioProtocol2011* thiss = const_cast<RadioProtocol2011*>(
            reinterpret_cast<const RadioProtocol2011*>(instance));
        thiss->_state = DISCONNECTED;
    }

    std::shared_ptr<CommModule> _commModule;
    CC1201* _radio;

    uint32_t _lastReceiveTime = 0;

    uint8_t _uid;
    State _state;

    std::vector<uint8_t> _reply;

    RtosTimer _replyTimer;
    RtosTimer _timeoutTimer;
};
