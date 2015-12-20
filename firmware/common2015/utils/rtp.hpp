#pragma once

#include <cstdint>

#if __BIG_ENDIAN
#define RTP_HEADER(port, subclass, ack, sfs)                                 \
    (((port & 0x0F) << 4) | ((subclass & 0x03) << 2) | ((ack & 0x01) << 1) | \
     (sfs & 0x01))
#else
#define RTP_HEADER(port, subclass, ack, sfs)                                \
    (((sfs & 0x01) << 7) | ((ack & 0x01) << 6) | ((subclass & 0x03) << 4) | \
     (port & 0x0F))
#endif

#define BASE_STATION_ADDR (1)
#define LOOPBACK_ADDR (127)

namespace rtp {
/// The number of bytes we must account for outsize of anything hardware related
static const unsigned int APP_HDR_SZ = 1;

/// The number of bytes we must account for when communicating the size of a raw
/// payload to any hardware device
static const unsigned int LINK_HDR_SZ = 2;

///
static const unsigned int MAX_DATA_SZ = 122;

/**
 * @brief      { port enumerations for different communication protocols }
 */
enum port {
    SINK = 0x00,
    LINK = 0x01,
    CONTROL = 0x02,
    SETPOINT = 0x03,
    GSTROBE = 0x04,
    DISCOVER = 0x05,
    LOG = 0x06,
    TCP = 0x07,
    LEGACY = 0x0E
};

/**
 * @brief      { Real-Time packet definition }
 */
struct packet {
    uint8_t total_size;

    union {  // [128 Bytes]
        struct {
            uint8_t raw[MAX_DATA_SZ + 6];  // [128 Bytes]
        };
        struct {  // [128 Bytes]
            union {
                struct {
                    uint8_t header[APP_HDR_SZ + LINK_HDR_SZ];
                };
                struct {
                    uint8_t payload_size;
                    uint8_t address;
                    union {
                        struct {
                            uint8_t header_link;
                        } __attribute__((packed));
                        struct {
#if __BIG_ENDIAN
                            uint8_t sfs : 1, ack : 1, subclass : 2, port : 4;
#else
                            uint8_t port : 4, subclass : 2, ack : 1, sfs : 1;
#endif
                        } __attribute__((packed));
                    };
                };
            };
            struct {
                uint8_t payload[MAX_DATA_SZ];  // [122 Bytes]
            };
            struct {
                uint8_t rssi;  // [1 Byte]
            };
            struct {
                uint8_t lqi;  // [1 Byte]
            };
        };
    };

    bool adjusted;

    packet(const packet& p) : total_size(p.total_size) {
        memcpy(raw, p.raw, total_size);
        resetSizes();
    }

    packet() : adjusted(false){};

    void adjustSizes() {
        if (adjusted == false) {
            payload_size += APP_HDR_SZ;
            total_size = payload_size + LINK_HDR_SZ;
            adjusted = true;
        }
    }

    void resetSizes() {
        if (adjusted == true) {
            payload_size -= APP_HDR_SZ;
            total_size = 0;
            adjusted = false;
        }
    }

    uint8_t ACK_Header() { return header_link; }
};
}
