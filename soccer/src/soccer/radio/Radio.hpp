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
    Radio() { channel_ = 0; }

    [[nodiscard]] virtual bool is_open() const = 0;

    virtual void send(
        const std::array<RobotIntent, kNumShells>& intents,
        const std::array<MotionSetpoint, kNumShells>& setpoints) = 0;
    virtual void receive() = 0;

    virtual void switch_team(bool blue_team) = 0;

    virtual void channel(int n) { channel_ = n; }

    int channel() const { return channel_; }

    bool has_reverse_packets() {
        std::lock_guard<std::mutex> lock(reverse_packets_mutex_);
        return reverse_packets_.size();
    }

    RobotStatus pop_reverse_packet() {
        std::lock_guard<std::mutex> lock(reverse_packets_mutex_);
        RobotStatus packet = reverse_packets_.front();
        reverse_packets_.pop_front();
        return packet;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(reverse_packets_mutex_);
        reverse_packets_.clear();
    }

protected:
    // A queue for the reverse packets as they come in.
    // Access to this queue should be controlled by locking the mutex.
    std::deque<RobotStatus> reverse_packets_;
    std::mutex reverse_packets_mutex_;

    int channel_;
};
