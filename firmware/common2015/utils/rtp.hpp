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

template <typename PACKET_TYPE>
void SerializeToVector(const PACKET_TYPE& pkt, std::vector<uint8_t>* buf) {
    const uint8_t* bytes = (const uint8_t*)&pkt;
    for (size_t i = 0; i < sizeof(PACKET_TYPE); i++) {
        buf->push_back(bytes[i]);
    }
}

template <typename PACKET_TYPE>
void SerializeToBuffer(const PACKET_TYPE& pkt, uint8_t* buf, size_t bufSize) {
    memcpy(buf, (const void*)&pkt, bufSize);
}

template <typename PACKET_TYPE>
bool DeserializeFromBuffer(PACKET_TYPE* pkt, uint8_t* buf, size_t bufSize) {
    if (bufSize < sizeof(PACKET_TYPE)) return false;

    memcpy(pkt, buf, bufSize);

    return true;
}

/**
 * @brief Port enumerations for different communication protocols.
 */
enum Port { SINK = 0, LINK = 1, CONTROL = 2, LEGACY = 3, PING = 4 };

struct header_data {
    enum Type { Control, Tuning, FirmwareUpdate, Misc };

    header_data(Port p = SINK) : address(0), port(p), type(Control){};

    uint8_t address;
    Port port : 4;
    Type type : 4;
};

// binary-packed version of Control.proto
struct ControlMessage {
    uint8_t uid;  // robot id
    int16_t bodyX;
    int16_t bodyY;
    int16_t bodyW;
    int8_t dribbler;
    uint8_t kickStrength;
    unsigned shootMode : 1;    // 0 = kick, 1 = chip
    unsigned triggerMode : 2;  // 0 = off, 1 = immediate, 2 = on break beam
    unsigned song : 2;         // 0 = stop, 1 = continue, 2 = GT fight song
} __attribute__((packed));

struct RobotStatusMessage {
    uint8_t uid;  // robot id
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
    packet(const std::string& s, Port p = SINK) : header(p) {
        for (char c : s) payload.push_back(c);
        payload.push_back('\0');
    }

    template <class T>
    packet(const std::vector<T>& v, Port p = SINK)
        : header(p) {
        for (T val : v) payload.push_back(val);
    }

    size_t size() const { return sizeof(header) + payload.size(); }

    /// deserialize a packet from a buffer
    template <class T>
    void recv(const std::vector<T>& v) {
        recv(v.data(), v.size());
    }

    /// deserialize a packet from a buffer
    void recv(const uint8_t* buffer, size_t size) {
        // check that the buffer is big enough
        if (size < sizeof(header)) return;

        // deserialize header.  skip first byte because it indicates size and
        // isn't part of the header
        header = *((header_data*)buffer + 1);

        // Everything after the header is payload data
        payload.clear();
        for (size_t i = sizeof(header) + 2; i < size; i++) {
            payload.push_back(buffer[i]);
        }
    }

    void pack(std::vector<uint8_t>* buffer) const {
        // first byte is total size (excluding the size byte)
        const uint8_t total_size = sizeof(header) + payload.size();
        buffer->reserve(total_size + 1);

        buffer->push_back(total_size);

        SerializeToVector(header, buffer);

        // payload
        buffer->insert(buffer->end(), payload.begin(), payload.end());
    }
};

// Packet sizes
constexpr unsigned int Forward_Size =
    sizeof(header_data) + 6 * sizeof(ControlMessage);
constexpr unsigned int Reverse_Size = 7;

}  // namespace rtp
