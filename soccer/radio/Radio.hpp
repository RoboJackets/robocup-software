#pragma once

#include <deque>
#include <mutex>

#include "RobotIntent.hpp"
#include "RobotStatus.hpp"
#include "motion/MotionSetpoint.hpp"

/**
 * @brief Sends and receives information to/from our robots.
 *
 * @details This is the abstract superclass for USBRadio and SimRadio, which do
 * the actual work - this just declares the interface.
 */
class Radio {
public:
    Radio() { _channel = 0; }

    [[nodiscard]] virtual bool isOpen() const = 0;

    virtual void send(
        const std::array<RobotIntent, Num_Shells>& intents,
        const std::array<MotionSetpoint, Num_Shells>& setpoints) = 0;
    virtual void receive() = 0;

    virtual void switchTeam(bool blueTeam) = 0;

    virtual void channel(int n) { _channel = n; }

    int channel() const { return _channel; }

    bool hasReversePackets() {
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        return _reversePackets.size();
    }

    RobotStatus popReversePacket() {
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        RobotStatus packet = _reversePackets.front();
        _reversePackets.pop_front();
        return packet;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.clear();
    }

protected:
    // A queue for the reverse packets as they come in.
    // Access to this queue should be controlled by locking the mutex.
    std::deque<RobotStatus> _reversePackets;
    std::mutex _reverse_packets_mutex;

    int _channel;
};
