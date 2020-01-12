#pragma once

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>

#include <mutex>
#include <deque>

/**
 * @brief Sends and receives information to/from our robots.
 *
 * @details This is the abstract superclass for USBRadio and SimRadio, which do
 * the actual work - this just declares the interface.
 */
class Radio : public Node {
public:
    Radio(Context* context, bool sim) { _channel = 0; }

    virtual bool isOpen() const = 0;
    virtual void send(Packet::RadioTx& radioTx) = 0;
    virtual void receive() = 0;

    virtual void switchTeam(bool blueTeam) = 0;

    virtual void channel(int n) { _channel = n; }

    void run() override;

    /** send out the radio data for the radio program */
    void sendRadioData();
    vector<int> getJoystickRobotIds();

    Radio* _radio;

    // True if we are running with a simulator.
    // This changes network communications.
    bool _simulation;

    // True if we are blue.
    // False if we are yellow.
    bool _blueTeam;

    // Board ID of the robot to manually control or -1 if none
    int _manualID;
    // Use multiple joysticks at once
    bool _multipleManual;

    int channel() const { return _channel; }

    bool hasReversePackets() {
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        return _reversePackets.size();
    }

    const Packet::RadioRx popReversePacket() {
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        Packet::RadioRx packet = std::move(_reversePackets.front());
        _reversePackets.pop_front();
        return packet;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.clear();
    }

protected:
    // A queue for the reverse packets as they come in through libusb.
    // Access to this queue should be controlled by locking the mutex.
    std::deque<Packet::RadioRx> _reversePackets;
    std::mutex _reverse_packets_mutex;

    int _channel;
};
