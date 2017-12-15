#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <limits>

namespace DebugCommunication {

    enum DebugResponse: uint8_t {
        DEBUG_RESPONSE_NONE=0,
        PIDError0 = 1,
        PIDError1,
        PIDError2,
        PIDError3,
        MotorDuty0,
        MotorDuty1,
        MotorDuty2,
        MotorDuty3,
        WheelVel0,
        WheelVel1,
        WheelVel2,
        WheelVel3,
        StallCounter0,
        StallCounter1,
        StallCounter2,
        StallCounter3,
        TargetWheelVel0,
        TargetWheelVel1,
        TargetWheelVel2,
        TargetWheelVel3,
        DEBUG_RESPONSE_LAST_PLACEHOLDER
    };

    constexpr std::array<std::pair<DebugResponse, float>,DEBUG_RESPONSE_LAST_PLACEHOLDER-1> RESPONSE_INFO= {
        std::make_pair(DebugResponse::PIDError0, 100.0),
        std::make_pair(DebugResponse::PIDError1, 100.0),
        std::make_pair(DebugResponse::PIDError2, 100.0),
        std::make_pair(DebugResponse::PIDError3, 100.0),
        std::make_pair(DebugResponse::MotorDuty0, 1),
        std::make_pair(DebugResponse::MotorDuty1, 1),
        std::make_pair(DebugResponse::MotorDuty2, 1),
        std::make_pair(DebugResponse::MotorDuty3, 1),
        std::make_pair(DebugResponse::WheelVel0, 100.0),
        std::make_pair(DebugResponse::WheelVel1, 100.0),
        std::make_pair(DebugResponse::WheelVel2, 100.0),
        std::make_pair(DebugResponse::WheelVel3, 100.0),
        std::make_pair(DebugResponse::StallCounter0, 1),
        std::make_pair(DebugResponse::StallCounter1, 1),
        std::make_pair(DebugResponse::StallCounter2, 1),
        std::make_pair(DebugResponse::StallCounter3, 1),
        std::make_pair(DebugResponse::TargetWheelVel0, 100),
        std::make_pair(DebugResponse::TargetWheelVel1, 100),
        std::make_pair(DebugResponse::TargetWheelVel2, 100),
        std::make_pair(DebugResponse::TargetWheelVel3, 100)
    };

    static_assert(RESPONSE_INFO.size() == DEBUG_RESPONSE_LAST_PLACEHOLDER-1, "Enums missing from RESPONSE_INFO");

    const std::array<float, DEBUG_RESPONSE_LAST_PLACEHOLDER> RESPONSE_TO_SCALING_FACTOR = [](){
        std::array<float, DEBUG_RESPONSE_LAST_PLACEHOLDER> a{};
        for (const auto& pair: RESPONSE_INFO) {
            if (pair.first!=DebugResponse::DEBUG_RESPONSE_NONE) {
                a[pair.first] = pair.second;
            }
        }
        return a;
    }();

    static int16_t debugResponseToValue(DebugResponse debugResponse, float value) {
        value*=RESPONSE_TO_SCALING_FACTOR[debugResponse];
        if (value>std::numeric_limits<int16_t>::max()) {
            return std::numeric_limits<int16_t>::max();
        } else if (value < std::numeric_limits<int16_t>::min()) {
            return std::numeric_limits<int16_t>::min();
        }
        return static_cast<int16_t>(value);
    };

    static float debugResponseValueToFloat(DebugResponse debugResponse, int16_t value) {
        return value/RESPONSE_TO_SCALING_FACTOR[debugResponse];
    }

    enum ConfigCommunication: uint8_t {
        CONFIG_COMMUNICATION_NONE=0,
        PID_P=1, PID_I, PID_D, CONFIG_COMMUNICATION_LAST_PLACEHOLDER
    };

    constexpr std::array<std::pair<ConfigCommunication, float>,CONFIG_COMMUNICATION_LAST_PLACEHOLDER-1> CONFIG_INFO = {
            std::make_pair(ConfigCommunication::PID_P, 1000.0f),
            std::make_pair(ConfigCommunication::PID_I, 1000.0f),
            std::make_pair(ConfigCommunication::PID_D, 1000.0f)
    };

    static_assert(CONFIG_INFO.size() == CONFIG_COMMUNICATION_LAST_PLACEHOLDER-1, "Enums missing from CONFIG_INFO");

    const std::array<float, CONFIG_COMMUNICATION_LAST_PLACEHOLDER> CONFIG_TO_SCALING_FACTOR = [](){
        std::array<float, CONFIG_COMMUNICATION_LAST_PLACEHOLDER> a{};
        for (const auto& pair: CONFIG_INFO) {
            if (pair.first!=ConfigCommunication::CONFIG_COMMUNICATION_LAST_PLACEHOLDER) {
                a[pair.first] = pair.second;
            }
        }
        return a;
    }();

    static int16_t configToValue(ConfigCommunication configCommunication, float value) {
        value*=CONFIG_TO_SCALING_FACTOR[configCommunication];
        if (value>std::numeric_limits<int16_t>::max()) {
            return std::numeric_limits<int16_t>::max();
        } else if (value < std::numeric_limits<int16_t>::min()) {
            return std::numeric_limits<int16_t>::min();
        }
        return static_cast<int16_t>(value);
    };

    static float configValueToFloat(ConfigCommunication configCommunication, int16_t value) {
        return value/CONFIG_TO_SCALING_FACTOR[configCommunication];
    }
}

namespace rtp {

// configured by the PKT_CFG1 register
constexpr uint8_t BROADCAST_ADDRESS = 0x00;

constexpr uint8_t BASE_STATION_ADDRESS = 0xFF - 1;
constexpr uint8_t ROBOT_ADDRESS = 0x01;  // All robots have the same address
constexpr uint8_t LOOPBACK_ADDRESS = 2;

// The value 0 is a valid robot id, so we have to choose something else to
// represent "null"
constexpr auto INVALID_ROBOT_UID = 0b111111;
constexpr auto ANY_ROBOT_UID = 0b111111 - 1;

template <typename PACKET_TYPE>
void serializeToVector(const PACKET_TYPE& pkt, std::vector<uint8_t>* buf) {
    const auto data = reinterpret_cast<const uint8_t*>(&pkt);
    buf->reserve(sizeof(PACKET_TYPE));
    for (size_t i = 0; i < sizeof(PACKET_TYPE); ++i) buf->push_back(data[i]);
}

// a hackish way of enforcing 'enum class' scopes without
// the bitfield restrictions
namespace PortTypeNamespace {
enum PortTypeEnum { SINK, LINK, CONTROL, LEGACY, PING };
}
using PortType = PortTypeNamespace::PortTypeEnum;

namespace MessageTypeNamespace {
enum MessageTypeEnum { CONTROL, TUNING, UPGRADE, MISC };
}
using MessageType = MessageTypeNamespace::MessageTypeEnum;

struct Header {
    Header(PortType p = PortType::SINK, MessageType t = MessageType::CONTROL)
        : address(INVALID_ROBOT_UID), port(p), type(t) {}

    uint8_t address;
    PortType port : 4;
    MessageType type : 4;
} __attribute__((packed));
static_assert(sizeof(Header) == 2, "sizeof(Header) is not what we expect");

// binary-packed version of Control.proto
struct ControlMessage {
    /** body{X,Y,W} are multiplied by this value before being sent over the
     * radio and must be then divided by this value on the receiving side. This
     * is to avoid loss of precision when sending float velocity values across
     * the air as ints.
     */
    static constexpr auto VELOCITY_SCALE_FACTOR = 1000;

//    uint8_t uid;
    int16_t bodyX;
    int16_t bodyY;
    int16_t bodyW;
    int8_t dribbler;
    uint8_t kickStrength;
    unsigned shootMode : 1;    // 0 = kick, 1 = chip
    unsigned triggerMode : 2;  // 0 = off, 1 = immediate, 2 = on break beam
//    unsigned debugStuff : 5;
    unsigned song : 2;         // 0 = stop, 1 = continue, 2 = GT fight song
} __attribute__((packed));
static_assert(sizeof(ControlMessage) == 9,
              "sizeof(ControlMessage) is not what we expect");

struct ConfMessage {
    static constexpr size_t length = 3;

    std::array<DebugCommunication::ConfigCommunication, length> keys{};
    std::array<int16_t, length> values{};
} __attribute__((packed));
static_assert(sizeof(ConfMessage) == 9,
              "sizeof(ConfMessage) is not what we expect");

struct DebugMessage {
    static constexpr size_t length = 3;
    std::array<DebugCommunication::DebugResponse, 3> keys;
};

static_assert(sizeof(DebugMessage) == 3,
              "sizeof(DebugMessage) is not what we expect");

struct RobotTxMessage {
    unsigned uid : 6;
    enum{ControlMessageType, ConfMessageType, DebugMessageType} messageType : 2;

    union RobotTxMessages {
        ControlMessage controlMessage;
        ConfMessage confMessage;
        DebugMessage debugMessage;
    } message;

} __attribute__((packed));


//
//template<int s, int t> struct check_size {
//    static_assert(s == t, "wrong size");
//};
//check_size<sizeof(RobotTxMessage), 10> ch;
static_assert(sizeof(RobotTxMessage) == 10,
              "sizeof(RobotTxMessage) is not what we expect");

struct RobotStatusMessage {
    /** @battVoltage is a direct reading from the mbed's ADC and is sent over
     * the air as-is.  Soccer must convert this reading into an actual voltage
     * value by multiplying it by the scale factor. The theoretical scale factor
     * is 0.100546875, but this has been adjusted after testing to the value
     * below.
     */
    static constexpr auto BATTERY_SCALE_FACTOR = 0.09884f;

    uint8_t uid;
    uint8_t battVoltage;
    unsigned motorErrors : 5;      // 0 = good, 1 = error
    unsigned ballSenseStatus : 1;  // 0 = no-ball, 1 = has-ball
    unsigned kickStatus : 1;       // 0 = uncharged, 1 = charged
    unsigned kickHealthy : 1;       // 0 = unhealthy, 1 = healthy
    unsigned fpgaStatus : 1;       // 0 = good, 1 = error

    static constexpr size_t debug_data_length = 0;
//    std::array<int16_t,debug_data_length> debug_data;
} __attribute__((packed));
static_assert(sizeof(RobotStatusMessage) == 4,
              "sizeof(RobotStatusMessage) is not what we expect");

// Packet sizes
static constexpr auto HeaderSize = sizeof(Header);
static constexpr auto ForwardSize = HeaderSize + 6 * sizeof(RobotTxMessage);
static constexpr auto ReverseSize = HeaderSize + sizeof(RobotStatusMessage);

/**
 * @brief Real-Time packet definition
 */
class Packet {
public:
    rtp::Header header;
    std::vector<uint8_t> payload;

    Packet(){};

    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    Packet(const std::vector<T>& v, PortType p = PortType::LEGACY) {
        assign(v);
    }

    template <
        typename T, std::size_t N,
        typename = std::enable_if_t<std::is_convertible<T, uint8_t>::value>>
    Packet(const std::array<T, N>& v, PortType p = PortType::LEGACY) {
        assign(v);
    }

    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    Packet(const std::initializer_list<T>& payloadBytes,
           PortType p = PortType::PING)
        : header(p, MessageType::MISC),
          payload(payloadBytes.begin(), payloadBytes.end()) {}

    Packet(const std::string& s, PortType p = PortType::SINK)
        : header(p, MessageType::MISC), payload(s.begin(), s.end()) {
        payload.push_back('\0');
    }

    /// Deserialize a packet from a buffer
    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    void assign(const std::vector<T>& buf) {
        // check that the buffer is big enough
        if (buf.size() >= HeaderSize) {
            // deserialize header
            header = *(reinterpret_cast<const Header*>(buf.data()));
            // set the payload bytes
            payload.assign(buf.begin() + HeaderSize, buf.end());
        }
    }

    template <
        typename T, std::size_t N,
        typename = std::enable_if_t<std::is_convertible<T, uint8_t>::value>>
    void assign(const std::array<T, N>& buf) {
        // enforce that the buffer is big enough
        static_assert(N >= HeaderSize,
                      "std::array<T,N> buffer is not large enough");
        // deserialize header
        header = *(reinterpret_cast<const Header*>(buf.data()));
        // set the payload bytes
        payload.assign(buf.begin() + HeaderSize, buf.end());
    }

    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    void pack(std::vector<T>* buf) const {
        buf->reserve(size());
        serializeToVector(header, buf);
        buf->insert(buf->end(), payload.begin(), payload.end());
    }

    size_t size() const { return HeaderSize + payload.size(); }
};

}  // namespace rtp
