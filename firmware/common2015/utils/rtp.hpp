#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace rtp {

/// Max packet size.  This is limited by the CC1201 buffer size.
static const unsigned int MAX_DATA_SZ = 120;

const uint8_t BROADCAST_ADDRESS = 0;
const uint8_t BASE_STATION_ADDRESS = 1;
const uint8_t LOOPBACK_ADDRESS = 2;

/**
 * @brief Port enumerations for different communication protocols.
 */
enum port { SINK = 0, LINK = 1, CONTROL = 2, LEGACY = 3, PING = 4 };

struct header_data {
    enum Type { Control, Tuning, FirmwareUpdate, Misc };

    header_data(uint8_t p = SINK) : address(0), port(p), type(Control){};

    uint8_t address;
    unsigned int port : 4;
    Type type : 4;
};

// binary-packed version of Control.proto
struct ControlMessage {
    uint8_t uniqueId;  // robot id
    int16_t bodyX;
    int16_t bodyY;
    int16_t bodyW;
    int8_t dribbler;
    uint8_t kickStrength;
    unsigned shootMode : 1;    // 0 = kick, 1 = chip
    unsigned triggerMode : 2;  // 0 = off, 1 = immediate, 2 = on break beam
    unsigned sing : 2;         // 0 = stop, 1 = continue, 2 = GT fight song
} __attribute__((packed));

struct RobotStatusMessage {
    uint8_t uniqueId;  // robot id
    uint8_t battVoltage;
};

/**
 * @brief Real-Time packet definition
 */
class packet {
public:
    rtp::header_data header;
    std::vector<uint8_t> payload;

    packet(){};
    packet(const std::string& s, uint8_t p = SINK) : header(p) {
        for (char c : s) payload.push_back(c);
        payload.push_back('\0');
    }

    template <class T>
    packet(const std::vector<T>& v, uint8_t p = SINK)
        : header(p) {
        for (T val : v) payload.push_back(val);
    }

    size_t size() const { return payload.size() + header.size(); }

    int port() const { return static_cast<int>(header.port); }
    template <class T>
    void port(T p) {
        header.port = static_cast<unsigned int>(p);
    }

    int address() { return header.address; }
    void address(int a) { header.address = static_cast<unsigned int>(a); }
};

template <struct PACKET_TYPE>
void SerializeToVector(const PACKET_TYPE& pkt, std::vector<uint8_t> buf) {
    uint8_t* bytes = (uint8_t*)pkt;
    for (size_t i = 0; i < sizeof(PACKET_TYPE); i++) {
        buf.push_back(bytes[i]);
    }
}

/// Serializes the message to the buffer and returns the number of bytes written
template <struct PACKET_TYPE>
void SerializeToBuffer(const PACKET_TYPE& pkt, uint8_t* buf, size_t bufSize) {
    memcpy(buf, (const void*)&pkt, bufSize);
}

}  // namespace rtp
